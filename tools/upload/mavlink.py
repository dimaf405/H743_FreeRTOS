"""固定版本的 MAVLink codec 与上传后 Application 身份验证。

本模块不推测固件版本；期望值只读取构建生成的 firmware identity 合同，并把
板端 HEARTBEAT/AUTOPILOT_VERSION 作为 reset 后确已进入目标应用的证据。
"""

from __future__ import annotations

import hashlib
import json
import pathlib
import sys

from .models import ApplicationIdentity, UploadError

MAVLINK_GCS_SYSTEM_ID = 255
MAVLINK_GCS_COMPONENT_ID = 190
MAVLINK_PX4_UPLOADER_COMPONENT_ID = 0
MAVLINK_APP_SYSTEM_ID = 1
MAVLINK_APP_COMPONENT_ID = 1
# Runtime-file digests from the size/SHA-256-pinned pymavlink 2.4.47 archive.
PINNED_PYMAVLINK_RUNTIME_SHA256 = {
    "__init__.py": "d902c5d877504a9098956ddf5c4a6321ba8e432815a78b279af4d8b79c939a5f",
    "dialects/__init__.py": "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
    "dialects/v20/__init__.py": "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
    "dialects/v20/common.py": "11761aba1f8eafcaceaebf02bb81a5836c823d83da28bcb405fc30ade11b02d7",
}

def load_expected_identity(path: pathlib.Path) -> tuple[int, int, tuple[int, ...]]:
    """读取并严格验证生成合同中的 wire-version、板版本和 8 字节 Git 身份。"""
    try:
        contract = json.loads(path.read_text(encoding="utf-8"))
        if contract.get("schema_version") != 1:
            raise ValueError("schema_version must be 1")
        flight_version = contract["mavlink"]["flight_version"]
        encoded = flight_version["encoded"]
        board_version = contract["board_version"]
        custom_version = tuple(contract["mavlink"]["flight_custom_version"])
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        raise UploadError(
            f"unable to load generated firmware identity contract {path}: {error}"
        ) from error
    if (
        isinstance(encoded, bool)
        or not isinstance(encoded, int)
        or encoded < 0
        or encoded > 0xFFFFFFFF
        or isinstance(board_version, bool)
        or not isinstance(board_version, int)
        or board_version < 0
        or board_version > 0xFFFFFFFF
        or len(custom_version) != 8
        or any(
            isinstance(value, bool)
            or not isinstance(value, int)
            or value < 0
            or value > 0xFF
            for value in custom_version
        )
    ):
        raise UploadError("generated firmware identity contract has invalid values")
    return encoded, board_version, custom_version


class MavlinkCodec:
    """不依赖 mavutil/pyserial 的 pinned pymavlink codec。

    串口所有权由上传状态机管理；这里仅负责编解码以及来源/身份/能力位核对。
    """

    def __init__(
        self,
        dialect: object,
        expected_flight_sw_version: int,
        expected_board_version: int,
        expected_custom_version: tuple[int, ...],
    ) -> None:
        self._dialect = dialect
        self._expected_flight_sw_version = expected_flight_sw_version
        self._expected_board_version = expected_board_version
        self._expected_custom_version = expected_custom_version
        self._encoder = dialect.MAVLink(  # type: ignore[attr-defined]
            None,
            srcSystem=MAVLINK_GCS_SYSTEM_ID,
            srcComponent=MAVLINK_GCS_COMPONENT_ID,
        )
        parameter_bytewise_capability = getattr(
            dialect, "MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE", None
        )
        if parameter_bytewise_capability is None:
            parameter_bytewise_capability = (
                dialect.MAV_PROTOCOL_CAPABILITY_PARAM_UNION  # type: ignore[attr-defined]
            )
        self._expected_capabilities = (
            dialect.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT  # type: ignore[attr-defined]
            | dialect.MAV_PROTOCOL_CAPABILITY_COMMAND_INT  # type: ignore[attr-defined]
            | parameter_bytewise_capability
            | dialect.MAV_PROTOCOL_CAPABILITY_MAVLINK2  # type: ignore[attr-defined]
        )
        # 必须同时声明 float 参数、COMMAND_INT、bytewise/union 参数和 MAVLink2；
        # 少任一位都说明运行的不是本次构建所约定的应用能力集合。

    def command_long(self, command: int, param1: float) -> bytes:
        """打包定向 1/1 的 MAVLink2 COMMAND_LONG，并按 uint8_t 回绕序号。"""
        message = self._encoder.command_long_encode(
            MAVLINK_APP_SYSTEM_ID,
            MAVLINK_APP_COMPONENT_ID,
            command,
            0,
            param1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        frame = message.pack(self._encoder, force_mavlink1=False)
        self._encoder.seq = (self._encoder.seq + 1) & 0xFF
        return frame

    def identify_request(self) -> bytes:
        """同时请求 HEARTBEAT(消息 0) 与 AUTOPILOT_VERSION，形成身份组合证据。"""
        request_message = self._dialect.MAV_CMD_REQUEST_MESSAGE
        return self.command_long(request_message, 0.0) + self.command_long(
            request_message, float(self._dialect.MAVLINK_MSG_ID_AUTOPILOT_VERSION)
        )

    def parse(self, payload: bytes) -> list[object]:
        """健壮解析一个接收批次；坏帧可跳过，但不会放宽后续字段校验。"""
        parser = self._dialect.MAVLink(None)
        parser.robust_parsing = True
        return list(parser.parse_buffer(payload) or [])

    @staticmethod
    def _source_is_application(message: object) -> bool:
        return (
            message.get_srcSystem() == MAVLINK_APP_SYSTEM_ID
            and message.get_srcComponent() == MAVLINK_APP_COMPONENT_ID
        )

    def application_identity(
        self,
        payload: bytes,
        *,
        require_target_build: bool,
    ) -> ApplicationIdentity | None:
        """识别兼容 Dima 应用，并按阶段决定是否核对本次目标 Git 身份。

        升级前板端通常运行旧提交，不能要求 custom version 等于待上传镜像；
        reset 后则必须精确匹配生成合同，才能证明启动的是本次构建。
        """
        heartbeat = None
        version = None
        for message in self.parse(payload):
            if not self._source_is_application(message):
                continue
            if message.get_type() == "HEARTBEAT":
                heartbeat = message
            elif message.get_type() == "AUTOPILOT_VERSION":
                version = message
        if heartbeat is None or version is None:
            return None
        if (
            heartbeat.type != self._dialect.MAV_TYPE_GROUND_ROVER
            or heartbeat.autopilot != self._dialect.MAV_AUTOPILOT_PX4
            or int(version.flight_sw_version) != self._expected_flight_sw_version
            or int(version.board_version) != self._expected_board_version
            or (
                int(version.capabilities) & self._expected_capabilities
            ) != self._expected_capabilities
            or int(version.uid) == 0
        ):
            return None
        # flight_sw_version 表示生成 manifest 声明的 PX4 线协议兼容版本；
        # flight_custom_version 才是随每次 Git 提交变化的目标镜像身份。
        if (
            require_target_build
            and tuple(int(value) for value in version.flight_custom_version)
            != self._expected_custom_version
        ):
            return None
        return ApplicationIdentity(
            uid=int(version.uid),
            flight_sw_version=int(version.flight_sw_version),
            board_version=int(version.board_version),
        )

    def reboot_ack(self, payload: bytes) -> tuple[bool | None, str]:
        """解析发往固定 uploader 255/0 的 Recovery mode=3 重启 ACK。"""
        command = self._dialect.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
        for message in self.parse(payload):
            if (
                message.get_type() != "COMMAND_ACK"
                or not self._source_is_application(message)
                or int(message.command) != command
            ):
                continue
            result = int(message.result)
            mode = int(message.result_param2)
            # Current firmware directs the ACK to the fixed PX4 uploader
            # frame's 255/0 source and reports Recovery mode 3.
            if int(message.target_system) != MAVLINK_GCS_SYSTEM_ID:
                continue
            if int(message.target_component) != MAVLINK_PX4_UPLOADER_COMPONENT_ID:
                continue
            if result == self._dialect.MAV_RESULT_ACCEPTED and mode == 3:
                return True, "MAVLink reboot ACK accepted (mode 3)"
            return False, f"MAVLink reboot was rejected (result={result}, mode={mode})"
        return None, "no matching MAVLink reboot ACK was received"

def resolve_mavlink_codec(
    tools_cache: pathlib.Path, identity_contract: pathlib.Path
) -> MavlinkCodec:
    """校验缓存运行时逐文件哈希后，从该唯一目录导入 pymavlink。"""
    expected_identity = load_expected_identity(identity_contract)
    tools_directory = pathlib.Path(__file__).resolve().parents[1] / "mavlink"
    bootstrap_directory = str(tools_directory)
    if bootstrap_directory not in sys.path:
        sys.path.insert(0, bootstrap_directory)
    try:
        from bootstrap_pymavlink import (  # type: ignore[import-not-found]
            BootstrapError as MavlinkBootstrapError,
            provision_pymavlink,
        )
    except ImportError as error:
        raise UploadError(
            "the pinned MAVLink bootstrap is unavailable under tools/mavlink"
        ) from error

    lock_path = tools_directory / "mavlink.lock.json"
    try:
        package_root = provision_pymavlink(tools_cache, lock_path)
    except MavlinkBootstrapError as error:
        raise UploadError(str(error)) from error

    # archive 级哈希之外再次固定实际会 import 的最小运行时闭包，防止缓存目录
    # 被局部替换后仍借包版本号通过检查。
    for relative, expected_digest in PINNED_PYMAVLINK_RUNTIME_SHA256.items():
        runtime_file = package_root / relative
        try:
            actual_digest = hashlib.sha256(runtime_file.read_bytes()).hexdigest()
        except OSError as error:
            raise UploadError(
                f"unable to verify cached pymavlink runtime {runtime_file}: {error}"
            ) from error
        if actual_digest != expected_digest:
            raise UploadError(
                f"cached pymavlink runtime integrity check failed: {runtime_file}"
            )

    package_parent = str(package_root.parent)
    if package_parent not in sys.path:
        sys.path.insert(0, package_parent)
    try:
        import pymavlink
        from pymavlink.dialects.v20 import common
    except ImportError as error:
        raise UploadError(f"unable to import pinned pymavlink: {error}") from error

    imported_root = pathlib.Path(pymavlink.__file__).resolve().parent
    if imported_root != package_root.resolve():
        raise UploadError(
            "the imported pymavlink package did not come from the pinned tools cache"
        )
    return MavlinkCodec(common, *expected_identity)
