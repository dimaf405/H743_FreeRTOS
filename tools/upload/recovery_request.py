"""Request a Dima application transition into MCUboot Recovery."""

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
    serial_exchange_bytes,
    serial_sequence_bytes,
)

# PX4-Autopilot Tools/px4_uploader.py sends fixed MAVLink v1 COMMAND_LONG
# frames. Both frames carry
# MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(param1=3): broadcast 0/0 first, then the
# conventional autopilot target 1/0. The exact MAVLink frame sequence is used
# for both automatic discovery and operator-selected ports; automatic discovery
# identifies the Dima application before issuing the Commander-arbitrated reboot.
PX4_MAVLINK_REBOOT_BROADCAST = bytes.fromhex(
    "fe2145ff004c00004040000000000000000000000000"
    "000000000000000000000000f600000000cc37"
)
PX4_MAVLINK_REBOOT_TARGETED = bytes.fromhex(
    "fe2172ff004c00004040000000000000000000000000"
    "000000000000000000000000f600010000536b"
)
PX4_REBOOT_ATTEMPTS = 3
PX4_REBOOT_SETTLE_SECONDS = 0.35

def px4_mavlink_reboot_stages(
    *, reset_each_attempt: bool = False
) -> tuple[SerialWriteStage, ...]:
    """Build PX4-style reboot writes for one open serial session."""
    stages: list[SerialWriteStage] = []
    for attempt in range(PX4_REBOOT_ATTEMPTS):
        stages.extend(
            (
                SerialWriteStage(
                    PX4_MAVLINK_REBOOT_BROADCAST,
                    # The explicit-port path matches PX4 and clears before
                    # every attempt.  The identified-application path clears
                    # only once so a late DENIED ACK survives retries.
                    reset_buffers=reset_each_attempt or attempt == 0,
                ),
                SerialWriteStage(
                    PX4_MAVLINK_REBOOT_TARGETED,
                    delay_seconds=PX4_REBOOT_SETTLE_SECONDS,
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
    remaining = None if deadline is None else deadline - time.monotonic()
    if remaining is not None and remaining <= 0:
        return False, None, "application probe deadline expired"
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
    return mavlink_sent, None, mavlink_error


def request_application_recovery(
    codec: MavlinkCodec,
    serial_backend: SerialBackend,
    port: str,
) -> tuple[bool, str]:
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


def request_explicit_port_recovery(
    serial_backend: SerialBackend,
    port: str,
) -> str:
    sequence = serial_sequence_bytes(
        serial_backend,
        port,
        px4_mavlink_reboot_stages(reset_each_attempt=True),
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
            f"REBOOT_REQUEST: unable to send the PX4 MAVLink reboot "
            f"sequence to {port}: "
            f"{sequence.error or 'no targeted reboot frame completed'}"
        )
    details = (
        f"PX4 explicit-port reboot writes={sequence.completed_stages}/"
        f"{PX4_REBOOT_ATTEMPTS * 2}"
    )
    if sequence.disconnected:
        details += "; application USB port disappeared during reboot"
    elif sequence.error:
        details += f"; serial sequence ended with {sequence.error}"
    return details
