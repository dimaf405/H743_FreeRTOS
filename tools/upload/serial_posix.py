"""Binary-safe POSIX serial exchange primitives."""

from __future__ import annotations

import errno
import os
import select
import time

try:
    import termios
    import tty
except ImportError:  # pragma: no cover - only POSIX hosts use these modules.
    termios = None
    tty = None

from .models import SerialSequenceResult, SerialWriteStage


def posix_serial_exchange_bytes(
    port: str, request: bytes, read_seconds: float
) -> tuple[bool, bytes, str]:
    """Perform a small raw serial exchange using only the Python standard library."""
    if termios is None or tty is None:
        return False, b"", "POSIX serial support is unavailable"

    try:
        descriptor = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as error:
        return False, b"", f"cannot open {port}: {error}"

    previous_attributes = None
    received = bytearray()
    sent = False
    try:
        previous_attributes = termios.tcgetattr(descriptor)
        tty.setraw(descriptor, when=termios.TCSANOW)
        attributes = termios.tcgetattr(descriptor)
        attributes[4] = termios.B115200
        attributes[5] = termios.B115200
        termios.tcsetattr(descriptor, termios.TCSANOW, attributes)
        termios.tcflush(descriptor, termios.TCIOFLUSH)

        write_deadline = time.monotonic() + 0.5
        offset = 0
        while offset < len(request) and time.monotonic() < write_deadline:
            remaining = max(0.0, write_deadline - time.monotonic())
            _, writable, _ = select.select([], [descriptor], [], remaining)
            if not writable:
                break
            try:
                offset += os.write(descriptor, request[offset:])
            except BlockingIOError:
                continue
        sent = offset == len(request)

        read_deadline = time.monotonic() + read_seconds
        while sent and time.monotonic() < read_deadline:
            remaining = max(0.0, read_deadline - time.monotonic())
            try:
                readable, _, _ = select.select([descriptor], [], [], remaining)
            except (OSError, ValueError):
                break
            if not readable:
                break
            try:
                chunk = os.read(descriptor, 4096)
            except BlockingIOError:
                continue
            except OSError:
                break
            if not chunk:
                break
            received.extend(chunk)
    except (OSError, termios.error) as error:
        return False, b"", f"serial exchange on {port} failed: {error}"
    finally:
        if previous_attributes is not None:
            try:
                termios.tcsetattr(descriptor, termios.TCSANOW, previous_attributes)
            except (OSError, termios.error):
                pass
        os.close(descriptor)

    if not sent:
        return False, bytes(received), f"serial write to {port} did not complete"
    return True, bytes(received), ""


def posix_serial_sequence_bytes(
    port: str,
    stages: tuple[SerialWriteStage, ...],
    read_seconds: float,
) -> SerialSequenceResult:
    """Run all staged writes through one raw POSIX descriptor."""
    if termios is None or tty is None:
        return SerialSequenceResult(
            False, 0, False, b"", "POSIX serial support is unavailable"
        )
    try:
        descriptor = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as error:
        return SerialSequenceResult(
            False, 0, False, b"", f"cannot open {port}: {error}"
        )

    previous_attributes = None
    received = bytearray()
    attempted = False
    completed_stages = 0
    disconnected = False
    failure = ""

    def collect_for(duration: float) -> None:
        deadline = time.monotonic() + max(0.0, duration)
        while True:
            remaining = max(0.0, deadline - time.monotonic())
            readable, _, _ = select.select([descriptor], [], [], remaining)
            if not readable:
                return
            chunk = os.read(descriptor, 4096)
            if not chunk:
                return
            received.extend(chunk)
            if time.monotonic() >= deadline:
                return

    try:
        previous_attributes = termios.tcgetattr(descriptor)
        tty.setraw(descriptor, when=termios.TCSANOW)
        attributes = termios.tcgetattr(descriptor)
        attributes[4] = termios.B115200
        attributes[5] = termios.B115200
        termios.tcsetattr(descriptor, termios.TCSANOW, attributes)

        for stage in stages:
            if stage.reset_buffers:
                termios.tcflush(descriptor, termios.TCIOFLUSH)

            attempted = True
            write_deadline = time.monotonic() + 0.5
            offset = 0
            while offset < len(stage.payload) and time.monotonic() < write_deadline:
                remaining = max(0.0, write_deadline - time.monotonic())
                _, writable, _ = select.select([], [descriptor], [], remaining)
                if not writable:
                    break
                try:
                    offset += os.write(descriptor, stage.payload[offset:])
                except BlockingIOError:
                    continue
            if offset != len(stage.payload):
                failure = f"serial write to {port} did not complete"
                break
            completed_stages += 1
            collect_for(stage.delay_seconds)

        if not failure:
            collect_for(read_seconds)
    except (OSError, ValueError, termios.error) as error:
        failure = f"serial sequence on {port} ended: {error}"
        disconnected = attempted and getattr(error, "errno", None) in {
            errno.ENODEV,
            errno.ENXIO,
            errno.EIO,
        }
    finally:
        if previous_attributes is not None:
            try:
                termios.tcsetattr(descriptor, termios.TCSANOW, previous_attributes)
            except (OSError, termios.error):
                pass
        try:
            os.close(descriptor)
        except OSError:
            pass

    return SerialSequenceResult(
        attempted,
        completed_stages,
        disconnected,
        bytes(received),
        failure,
    )
