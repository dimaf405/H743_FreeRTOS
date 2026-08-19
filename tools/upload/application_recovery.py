"""Dima legacy and PX4-compatible application-to-Recovery bridges."""

from __future__ import annotations

import time

from .mavlink import MavlinkCodec
from .models import (
    ApplicationIdentity,
    SerialBackend,
    SerialWriteStage,
    UploadError,
)
from .serial_io import (
    serial_exchange,
    serial_exchange_bytes,
    serial_sequence_bytes,
)

LEGACY_APP_IDENTIFY_TOKEN = "DIMA_ROVER_APP_V1"
LEGACY_APP_REBOOT_ACK = "DIMA_REBOOTING_BOOTLOADER"
LEGACY_APP_REBOOT_DENIED = "DIMA_REBOOT_DENIED_ARMED"
LEGACY_APP_IDENTIFY_REQUEST = b"\r\r\rdima identify\n"
LEGACY_APP_REBOOT_REQUEST = b"\r\r\rreboot -b\n"
# PX4-Autopilot Tools/px4_uploader.py sends fixed MAVLink v1 COMMAND_LONG
# frames followed by the NSH reboot command. Both frames carry
# MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(param1=3): broadcast 0/0 first, then the
# conventional autopilot target 1/0. The exact MAVLink+NSH sequence is used
# only when the operator explicitly selected a serial port; automatic discovery
# still identifies the Dima application before issuing a Commander-arbitrated
# MAVLink reboot.
PX4_MAVLINK_REBOOT_BROADCAST = bytes.fromhex(
    "fe2145ff004c00004040000000000000000000000000"
    "000000000000000000000000f600000000cc37"
)
PX4_MAVLINK_REBOOT_TARGETED = bytes.fromhex(
    "fe2172ff004c00004040000000000000000000000000"
    "000000000000000000000000f600010000536b"
)
PX4_NSH_INIT = b"\r\r\r"
PX4_NSH_REBOOT_BOOTLOADER = b"reboot -b\n"
PX4_REBOOT_ATTEMPTS = 3
PX4_REBOOT_SETTLE_SECONDS = 0.35
PX4_BOOTSTRAP_MAVLINK_SETTLE_SECONDS = 0.1
PX4_BOOTSTRAP_NSH_INIT_SETTLE_SECONDS = 0.05
PX4_BOOTSTRAP_NSH_REBOOT_SETTLE_SECONDS = 0.2

def px4_mavlink_reboot_stages() -> tuple[SerialWriteStage, ...]:
    """PX4-style automatic reboot writes, kept on one open serial session."""
    stages: list[SerialWriteStage] = []
    for attempt in range(PX4_REBOOT_ATTEMPTS):
        stages.extend(
            (
                SerialWriteStage(
                    PX4_MAVLINK_REBOOT_BROADCAST,
                    # PX4 clears buffers before every attempt because it never
                    # inspects application replies.  Dima clears only once so
                    # a late DENIED ACK cannot be discarded between retries.
                    reset_buffers=attempt == 0,
                ),
                SerialWriteStage(
                    PX4_MAVLINK_REBOOT_TARGETED,
                    delay_seconds=PX4_REBOOT_SETTLE_SECONDS,
                ),
            )
        )
    return tuple(stages)


def px4_bootstrap_reboot_stages() -> tuple[SerialWriteStage, ...]:
    """Exact PX4 MAVLink+NSH bootloader kick for an operator-selected port."""
    stages: list[SerialWriteStage] = []
    for _ in range(PX4_REBOOT_ATTEMPTS):
        stages.extend(
            (
                SerialWriteStage(
                    PX4_MAVLINK_REBOOT_BROADCAST,
                    reset_buffers=True,
                ),
                SerialWriteStage(
                    PX4_MAVLINK_REBOOT_TARGETED,
                    delay_seconds=PX4_BOOTSTRAP_MAVLINK_SETTLE_SECONDS,
                ),
                SerialWriteStage(
                    PX4_NSH_INIT,
                    delay_seconds=PX4_BOOTSTRAP_NSH_INIT_SETTLE_SECONDS,
                ),
                SerialWriteStage(
                    PX4_NSH_REBOOT_BOOTLOADER,
                    delay_seconds=PX4_BOOTSTRAP_NSH_REBOOT_SETTLE_SECONDS,
                ),
            )
        )
    return tuple(stages)

def try_application_identify(
    codec: MavlinkCodec,
    serial_backend: SerialBackend,
    port: str,
    deadline: float | None = None,
) -> tuple[bool, ApplicationIdentity | None, str]:
    # Keep one upgrade bridge for boards still running the pre-MAVLink console.
    remaining = None if deadline is None else deadline - time.monotonic()
    if remaining is not None and remaining <= 0:
        return False, None, "application probe deadline expired"
    legacy_read_seconds = min(0.45, remaining) if remaining is not None else 0.45
    sent, output = serial_exchange(
        serial_backend,
        port,
        LEGACY_APP_IDENTIFY_REQUEST,
        read_seconds=legacy_read_seconds,
    )
    if sent and LEGACY_APP_IDENTIFY_TOKEN in output:
        return sent, ApplicationIdentity(protocol="legacy"), output

    remaining = None if deadline is None else deadline - time.monotonic()
    if remaining is not None and remaining <= 0:
        return sent, None, output or "application probe deadline expired"
    mavlink_read_seconds = min(0.8, remaining) if remaining is not None else 0.8
    mavlink_sent, binary_output, mavlink_error = serial_exchange_bytes(
        serial_backend,
        port,
        codec.identify_request(),
        read_seconds=mavlink_read_seconds,
    )
    if mavlink_sent:
        identity = codec.application_identity(binary_output)
        if identity is not None:
            return True, identity, identity.summary()
        mavlink_error = "no matching Dima Rover MAVLink identity was received"
    details = output or mavlink_error
    return sent or mavlink_sent, None, details


def request_application_recovery(
    codec: MavlinkCodec,
    serial_backend: SerialBackend,
    port: str,
    identity: ApplicationIdentity,
) -> tuple[bool, str]:
    if identity.protocol == "legacy":
        sent, output = serial_exchange(
            serial_backend, port, LEGACY_APP_REBOOT_REQUEST, read_seconds=0.45
        )
        if not sent:
            raise UploadError(
                f"REBOOT_REQUEST: unable to send reboot -b to {port}: {output}"
            )
        if LEGACY_APP_REBOOT_DENIED in output:
            raise UploadError(
                "REBOOT_REQUEST: the application refused bootloader reboot because it is armed"
            )
        if LEGACY_APP_REBOOT_ACK in output:
            return True, "legacy reboot ACK accepted"
        return False, output or "legacy reboot -b sent without an ACK"

    sequence = serial_sequence_bytes(
        serial_backend,
        port,
        px4_mavlink_reboot_stages(),
        read_seconds=0.25,
    )
    if (
        not sequence.attempted
        or (
            sequence.completed_stages < 2
            and not sequence.disconnected
        )
    ):
        raise UploadError(
            f"REBOOT_REQUEST: unable to send PX4 MAVLink reboot sequence to "
            f"{port}: {sequence.error or 'no targeted reboot frame completed'}"
        )
    accepted, details = codec.reboot_ack(sequence.output)
    if accepted is False:
        raise UploadError(f"REBOOT_REQUEST: {details}")
    sequence_details = (
        f"PX4 MAVLink reboot writes={sequence.completed_stages}/"
        f"{PX4_REBOOT_ATTEMPTS * 2}"
    )
    if sequence.disconnected:
        sequence_details += "; application USB port disappeared during reboot"
    elif sequence.error:
        sequence_details += f"; serial sequence ended with {sequence.error}"
    if accepted is True:
        return True, f"{details}; {sequence_details}"
    return False, f"{details}; {sequence_details}"


def request_px4_bootstrap_recovery(
    serial_backend: SerialBackend,
    port: str,
) -> str:
    sequence = serial_sequence_bytes(
        serial_backend,
        port,
        px4_bootstrap_reboot_stages(),
        read_seconds=0.0,
    )
    if (
        not sequence.attempted
        or (
            sequence.completed_stages < 2
            and not sequence.disconnected
        )
    ):
        raise UploadError(
            f"REBOOT_REQUEST: unable to send the PX4 MAVLink/NSH reboot "
            f"sequence to {port}: "
            f"{sequence.error or 'no targeted reboot frame completed'}"
        )
    details = (
        f"PX4 explicit-port reboot writes={sequence.completed_stages}/"
        f"{PX4_REBOOT_ATTEMPTS * 4}"
    )
    if sequence.disconnected:
        details += "; application USB port disappeared during reboot"
    elif sequence.error:
        details += f"; serial sequence ended with {sequence.error}"
    return details

