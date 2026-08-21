"""Pinned MAVLink codec and application identity verification."""

from __future__ import annotations

import hashlib
import pathlib
import sys

from .models import ApplicationIdentity, UploadError

MAVLINK_GCS_SYSTEM_ID = 255
MAVLINK_GCS_COMPONENT_ID = 190
MAVLINK_PX4_UPLOADER_COMPONENT_ID = 0
MAVLINK_APP_SYSTEM_ID = 1
MAVLINK_APP_COMPONENT_ID = 1
DIMA_FLIGHT_SW_VERSION = 0x00010000
DIMA_BOARD_VERSION = 1
# Runtime-file digests from the size/SHA-256-pinned pymavlink 2.4.47 archive.
PINNED_PYMAVLINK_RUNTIME_SHA256 = {
    "__init__.py": "d902c5d877504a9098956ddf5c4a6321ba8e432815a78b279af4d8b79c939a5f",
    "dialects/__init__.py": "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
    "dialects/v20/__init__.py": "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
    "dialects/v20/common.py": "11761aba1f8eafcaceaebf02bb81a5836c823d83da28bcb405fc30ade11b02d7",
}

class MavlinkCodec:
    """Pinned pymavlink codec without mavutil or a pyserial dependency."""

    def __init__(self, dialect: object) -> None:
        self._dialect = dialect
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

    def command_long(self, command: int, param1: float) -> bytes:
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
        request_message = self._dialect.MAV_CMD_REQUEST_MESSAGE
        return self.command_long(request_message, 0.0) + self.command_long(
            request_message, float(self._dialect.MAVLINK_MSG_ID_AUTOPILOT_VERSION)
        )

    def parse(self, payload: bytes) -> list[object]:
        parser = self._dialect.MAVLink(None)
        parser.robust_parsing = True
        return list(parser.parse_buffer(payload) or [])

    @staticmethod
    def _source_is_application(message: object) -> bool:
        return (
            message.get_srcSystem() == MAVLINK_APP_SYSTEM_ID
            and message.get_srcComponent() == MAVLINK_APP_COMPONENT_ID
        )

    def application_identity(self, payload: bytes) -> ApplicationIdentity | None:
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
            or int(version.flight_sw_version) != DIMA_FLIGHT_SW_VERSION
            or int(version.board_version) != DIMA_BOARD_VERSION
            or (
                int(version.capabilities) & self._expected_capabilities
            ) != self._expected_capabilities
            or int(version.uid) == 0
        ):
            return None
        return ApplicationIdentity(
            uid=int(version.uid),
            flight_sw_version=int(version.flight_sw_version),
            board_version=int(version.board_version),
        )

    def reboot_ack(self, payload: bytes) -> tuple[bool | None, str]:
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

def resolve_mavlink_codec(tools_cache: pathlib.Path) -> MavlinkCodec:
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
    return MavlinkCodec(common)
