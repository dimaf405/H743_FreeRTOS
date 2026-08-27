"""签名镜像上传、MCUboot TEST 请求与应用重启流程。"""

from __future__ import annotations

import argparse
import pathlib

from .mavlink import resolve_mavlink_codec
from .mcumgr import (
    has_active_confirmed_image,
    has_secondary_image,
    image_hash,
    mcumgr_image_path,
    run_mcumgr,
)
from .models import (
    DEFAULT_MAX_WINDOW,
    DEFAULT_SERIAL_MTU,
    DEFAULT_USB_CDC_BAUD,
    UploadError,
    reset_stage_timing,
    stage,
)
from .endpoint_preflight import endpoint_preflight
from .endpoint_wait import (
    wait_for_application,
    wait_for_recovery,
)
from .runtime import resolve_mcumgr

def main() -> int:
    """执行工具解析、端点绑定、Secondary 上传、TEST、复位与应用身份闭环。"""
    reset_stage_timing()
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", type=pathlib.Path)
    parser.add_argument("--imgtool", type=pathlib.Path)
    parser.add_argument("--identity-contract", type=pathlib.Path, required=True)
    parser.add_argument("--mcumgr", default="mcumgr")
    parser.add_argument(
        "--tools-cache",
        type=pathlib.Path,
        default=pathlib.Path.home() / ".cache" / "dima-rover" / "host-tools",
    )
    parser.add_argument("--port")
    parser.add_argument("--wait-seconds", type=int, default=60)
    parser.add_argument("--baud", type=int, default=DEFAULT_USB_CDC_BAUD)
    parser.add_argument("--mtu", type=int, default=DEFAULT_SERIAL_MTU)
    parser.add_argument("--max-window", type=int, default=DEFAULT_MAX_WINDOW)
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--force", action="store_true")
    arguments = parser.parse_args()

    if arguments.wait_seconds <= 0:
        parser.error("--wait-seconds must be positive")
    if arguments.baud <= 0:
        parser.error("--baud must be positive")
    if arguments.mtu < 128 or arguments.mtu > 512:
        parser.error("--mtu must be between 128 and 512 for this MCUboot build")
    if arguments.max_window < 1 or arguments.max_window > 3:
        parser.error("--max-window must be between 1 and 3 for the 2048-byte RX ring")
    tools_cache = arguments.tools_cache.expanduser()
    stage("HOST", "resolving pinned upload tools and MAVLink codec")
    runtime = resolve_mcumgr(arguments.mcumgr, arguments.port, tools_cache)
    codec = resolve_mavlink_codec(
        tools_cache, arguments.identity_contract.resolve()
    )
    stage(
        "HOST_READY",
        f"mcumgr={pathlib.Path(runtime.executable).name} "
        f"transport={runtime.serial_backend.value}",
    )
    if arguments.preflight_only:
        endpoint_preflight(
            runtime,
            codec,
            arguments.port,
            arguments.wait_seconds,
            arguments.baud,
            arguments.mtu,
        )
        return 0

    if arguments.image is None or arguments.imgtool is None:
        parser.error("--image and --imgtool are required unless --preflight-only is used")

    image = arguments.image.resolve()
    digest = image_hash(arguments.imgtool.resolve(), image)
    stage("SIGN_VERIFY", f"signed image hash={digest}")
    port, initial_list, initial_identity, initial_binding = wait_for_recovery(
        runtime,
        codec,
        arguments.port,
        arguments.wait_seconds,
        arguments.baud,
        arguments.mtu,
    )
    already_active_confirmed = has_active_confirmed_image(initial_list, digest)
    if already_active_confirmed and not arguments.force:
        stage(
            "ALREADY_INSTALLED",
            "the signed image is already active and confirmed; use --force to rewrite it",
        )
        stage("RESET", "returning from MCUboot Recovery to the application")
        run_mcumgr(
            runtime.executable,
            port,
            "reset",
            baud=arguments.baud,
            mtu=arguments.mtu,
        )
        wait_for_application(
            runtime,
            codec,
            port,
            arguments.wait_seconds,
            initial_identity,
            initial_binding,
        )
        stage("COMPLETE", "firmware is already installed and the application is running")
        return 0

    upload_image = mcumgr_image_path(runtime, image)

    # Bootloader 用 2048 字节环接收串口帧；更大窗口完成持续板测前，生产默认保持停等窗口 1。
    stage(
        "UPLOAD_SECONDARY",
        f"uploading {image.stat().st_size} bytes with mtu={arguments.mtu} "
        f"window={arguments.max_window}",
    )
    run_mcumgr(
        runtime.executable,
        port,
        "image",
        "upload",
        "-n",
        "2",
        "--maxwinsize",
        str(arguments.max_window),
        upload_image,
        timeout_seconds=900,
        baud=arguments.baud,
        mtu=arguments.mtu,
        measure_bytes=image.stat().st_size,
    )
    # 强制重写当前 active hash 时，MCUboot 按 hash 查找会先匹配 Primary；仅该歧义场景
    # 需要在 TEST 前额外用 slot 1 证明 Secondary 确已写入。
    if already_active_confirmed:
        stage("VERIFY_SECONDARY", "verifying the forced Secondary rewrite")
        secondary_list = run_mcumgr(
            runtime.executable,
            port,
            "image",
            "list",
            expect_images=True,
            baud=arguments.baud,
            mtu=arguments.mtu,
        )
        if not has_secondary_image(secondary_list, digest):
            raise UploadError(
                "SECONDARY_HASH_MISMATCH: the forced image was not found in slot 1"
            )
    stage("TEST", f"marking {digest} as the test image")
    run_mcumgr(
        runtime.executable,
        port,
        "image",
        "test",
        digest,
        baud=arguments.baud,
        mtu=arguments.mtu,
    )
    stage("RESET", "resetting into the test image")
    run_mcumgr(
        runtime.executable,
        port,
        "reset",
        baud=arguments.baud,
        mtu=arguments.mtu,
    )
    wait_for_application(
        runtime,
        codec,
        port,
        arguments.wait_seconds,
        initial_identity,
        initial_binding,
    )
    stage("COMPLETE", "upload, swap, and application restart passed")
    return 0
