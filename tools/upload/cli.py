"""Command-line workflow for upload, swap, confirmation, and restart."""

from __future__ import annotations

import argparse
import pathlib
import time

from .application_recovery import request_application_recovery
from .mavlink import resolve_mavlink_codec
from .mcumgr import (
    has_active_confirmed_image,
    has_pending_secondary_image,
    has_secondary_image,
    has_unconfirmed_active_image,
    image_hash,
    mcumgr_image_path,
    run_mcumgr,
)
from .models import (
    DEFAULT_CONFIRM_WAIT_SECONDS,
    DEFAULT_MAX_WINDOW,
    DEFAULT_SERIAL_MTU,
    DEFAULT_USB_CDC_BAUD,
    UploadError,
    stage,
)
from .recovery import (
    endpoint_preflight,
    wait_for_application,
    wait_for_recovery,
    wait_for_recovery_endpoint,
)
from .runtime import resolve_mcumgr

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", type=pathlib.Path)
    parser.add_argument("--imgtool", type=pathlib.Path)
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
    parser.add_argument(
        "--confirm-wait-seconds",
        type=int,
        default=DEFAULT_CONFIRM_WAIT_SECONDS,
    )
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--skip-confirm-verification", action="store_true")
    arguments = parser.parse_args()

    if arguments.wait_seconds <= 0:
        parser.error("--wait-seconds must be positive")
    if arguments.baud <= 0:
        parser.error("--baud must be positive")
    if arguments.mtu < 128 or arguments.mtu > 512:
        parser.error("--mtu must be between 128 and 512 for this MCUboot build")
    if arguments.max_window < 1 or arguments.max_window > 3:
        parser.error("--max-window must be between 1 and 3 for the 2048-byte RX ring")
    if arguments.confirm_wait_seconds <= 0:
        parser.error("--confirm-wait-seconds must be positive")

    tools_cache = arguments.tools_cache.expanduser()
    runtime = resolve_mcumgr(arguments.mcumgr, arguments.port, tools_cache)
    codec = resolve_mavlink_codec(tools_cache)
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
    if has_unconfirmed_active_image(initial_list):
        raise UploadError(
            "ACTIVE_IMAGE_UNCONFIRMED: refusing to overwrite Secondary while "
            "the current test image still depends on it for rollback"
        )
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

    # The bootloader receives serial frames in a 2048-byte ring. Stop-and-wait
    # remains the production default until a larger window passes sustained
    # real-board acceptance.
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
    # An explicitly forced rewrite of the active hash is ambiguous to MCUboot's
    # hash lookup because Primary matches first.  Only that uncommon case needs
    # a separate Secondary proof before setting pending.
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
    pending_list = run_mcumgr(
        runtime.executable,
        port,
        "image",
        "test",
        digest,
        expect_images=True,
        baud=arguments.baud,
        mtu=arguments.mtu,
    )
    stage("VERIFY_SECONDARY", "verifying hash and pending state from TEST response")
    if not has_pending_secondary_image(pending_list, digest):
        raise UploadError(
            "PENDING_STATE_MISMATCH: uploaded image was not pending in slot 1"
        )
    stage("RESET", "resetting into the test image")
    run_mcumgr(
        runtime.executable,
        port,
        "reset",
        baud=arguments.baud,
        mtu=arguments.mtu,
    )
    application_port, application_identity, application_binding = wait_for_application(
        runtime,
        codec,
        port,
        arguments.wait_seconds,
        initial_identity,
        initial_binding,
    )
    if arguments.skip_confirm_verification:
        stage(
            "COMPLETE",
            "upload and swap completed; application confirmation was not probed",
        )
        return 0

    stage(
        "HEALTH_CONFIRM",
        f"waiting {arguments.confirm_wait_seconds}s for application health confirmation",
    )
    time.sleep(arguments.confirm_wait_seconds)
    reboot_acknowledged, reboot_output = request_application_recovery(
        codec,
        runtime.serial_backend,
        application_port,
        application_identity,
    )
    if reboot_output:
        print(reboot_output)
    if not reboot_acknowledged:
        stage(
            "REBOOT_REQUEST",
            "automatic reboot sequence sent; ACK was not required; waiting "
            "for MCUboot on the same physical USB device",
        )
    confirm_port, confirm_list = wait_for_recovery_endpoint(
        runtime,
        application_binding,
        arguments.wait_seconds,
        arguments.baud,
        arguments.mtu,
    )
    confirmed = has_active_confirmed_image(confirm_list, digest)
    stage("RESET", "returning from confirmation probe to the application")
    run_mcumgr(
        runtime.executable,
        confirm_port,
        "reset",
        baud=arguments.baud,
        mtu=arguments.mtu,
    )
    wait_for_application(
        runtime,
        codec,
        confirm_port,
        arguments.wait_seconds,
        application_identity,
        application_binding,
    )
    if not confirmed:
        raise UploadError(
            "IMAGE_NOT_CONFIRMED: the uploaded image did not become active and confirmed"
        )
    stage("COMPLETE", "upload, swap, health confirmation, and application restart passed")
    return 0
