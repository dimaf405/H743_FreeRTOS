"""签名镜像 TLV 检查、MCUboot image 状态解析与 Apache mcumgr 有界命令客户端。"""

from __future__ import annotations

import json
import os
import pathlib
import queue
import re
import shlex
import subprocess
import sys
import threading
import time

from .models import (
    DEFAULT_SERIAL_MTU,
    DEFAULT_USB_CDC_BAUD,
    HostPlatform,
    ImageState,
    McumgrRuntime,
    UploadError,
    decode_output,
)

IMAGE_SHA256_TLV = 0x10
MCUMGR_ERROR_RE = re.compile(
    r"^Error:[ \t]*([+-]?[0-9]+)[ \t]*\r?$", re.MULTILINE
)
MCUMGR_IMAGES_RE = re.compile(r"^Images:[ \t]*\r?$", re.MULTILINE)

def image_hash(imgtool: pathlib.Path, image: pathlib.Path) -> str:
    """从 imgtool 解析唯一 32 字节 SHA-256 TLV；不是对整个 signed BIN 再求文件散列。"""
    if not image.is_file():
        raise UploadError(f"signed upload image does not exist: {image}")
    if not imgtool.is_file():
        raise UploadError(f"imgtool does not exist: {imgtool}")

    completed = subprocess.run(
        [sys.executable, str(imgtool), "dumpinfo", "--format", "json", str(image)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if completed.returncode != 0:
        details = decode_output(completed.stderr or completed.stdout)
        raise UploadError(f"unable to inspect signed image: {details}")

    try:
        metadata = json.loads(completed.stdout)
        tlvs = metadata["tlv_area"]["tlvs"]
    except (KeyError, TypeError, json.JSONDecodeError) as error:
        raise UploadError("imgtool returned malformed signed-image metadata") from error

    if not isinstance(tlvs, list):
        raise UploadError("imgtool returned malformed signed-image TLVs")
    sha_tlvs = [
        tlv
        for tlv in tlvs
        if isinstance(tlv, dict) and tlv.get("type") == IMAGE_SHA256_TLV
    ]
    if len(sha_tlvs) != 1:
        raise UploadError(
            f"signed image must contain exactly one SHA-256 TLV; found {len(sha_tlvs)}"
        )

    sha_tlv = sha_tlvs[0]
    digest = sha_tlv.get("data")
    if sha_tlv.get("len") != 32 or not isinstance(digest, str):
        raise UploadError("signed-image SHA-256 TLV is not 32 bytes")
    if re.fullmatch(r"[0-9a-fA-F]{64}", digest) is None:
        raise UploadError("signed-image SHA-256 TLV is not 64 hexadecimal characters")
    return digest.lower()


def wsl_windows_path(path: pathlib.Path) -> str:
    completed = subprocess.run(
        ["wslpath", "-w", str(path.resolve())],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    converted = decode_output(completed.stdout)
    if completed.returncode != 0 or not converted:
        raise UploadError(f"cannot convert image path for Windows mcumgr: {path}")
    return converted


def mcumgr_image_path(runtime: McumgrRuntime, path: pathlib.Path) -> str:
    if (
        runtime.host_platform == HostPlatform.WSL
        and runtime.executable_is_windows
    ):
        return wsl_windows_path(path)
    return str(path.resolve())


def mcumgr_command(
    executable: str,
    port: str,
    *arguments: str,
    baud: int = DEFAULT_USB_CDC_BAUD,
    mtu: int = DEFAULT_SERIAL_MTU,
) -> list[str]:
    return [
        executable,
        "--conntype",
        "serial",
        "--connstring",
        f"dev={port},baud={baud},mtu={mtu}",
        *arguments,
    ]


def protocol_error(output: str) -> str | None:
    for match in MCUMGR_ERROR_RE.finditer(output):
        if int(match.group(1)) != 0:
            return match.group(0)
    return None


def _stop_owned_mcumgr(
    process: subprocess.Popen[bytes], reader: threading.Thread | None
) -> None:
    """终止本次上传器创建的 mcumgr，并等待输出线程释放管道和串口句柄。"""
    # Windows 上 mcumgr 持有排他的 COM 句柄；只 kill 不 wait 会让取消上传后的
    # 句柄释放时机不确定。这里严格限定为当前 Popen 子进程，不扫描或结束其他用户进程。
    if process.poll() is None:
        try:
            process.kill()
        except OSError:
            # 子进程可能恰好在 poll 与 kill 之间退出，后续 wait 仍负责回收句柄。
            pass
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        # kill 后仍未退出属于宿主异常；再次请求终止并继续执行有界回收，不能让
        # 上传器永久卡在清理路径。
        try:
            process.kill()
        except OSError:
            pass
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            pass

    if reader is None or reader.ident is None:
        return
    reader.join(timeout=1.0)
    if reader.is_alive() and process.stdout is not None:
        # 极端情况下 Windows 管道读仍未返回；关闭父端 pipe 使 daemon reader
        # 尽快退出。reader 不参与协议状态，因此不得阻塞上传器关停。
        process.stdout.close()
        reader.join(timeout=1.0)


def try_image_list(
    executable: str, port: str, baud: int, mtu: int
) -> tuple[bool, str]:
    command = mcumgr_command(
        executable,
        port,
        "--timeout",
        "0.5",
        "--tries",
        "1",
        "image",
        "list",
        baud=baud,
        mtu=mtu,
    )
    try:
        completed = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=1.5,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return False, "image list timed out"
    except OSError as error:
        return False, f"unable to start mcumgr: {error}"
    output = decode_output(completed.stdout)
    success = (
        completed.returncode == 0
        and protocol_error(output) is None
        and MCUMGR_IMAGES_RE.search(output) is not None
    )
    return success, output

def parse_image_states(output: str) -> list[ImageState]:
    """把 mcumgr 文本解析为 image/slot/version/flags/hash 结构，未知行不影响已知字段。"""
    states: list[ImageState] = []
    current: ImageState | None = None
    for raw_line in output.splitlines():
        line = raw_line.strip()
        header = re.fullmatch(r"image=([0-9]+) slot=([0-9]+)", line)
        if header is not None:
            current = ImageState(
                image=int(header.group(1)),
                slot=int(header.group(2)),
            )
            states.append(current)
            continue
        if current is None:
            continue
        if line.startswith("version:"):
            current.version = line.partition(":")[2].strip()
        elif line.startswith("flags:"):
            current.flags = set(line.partition(":")[2].strip().split())
        elif line.startswith("hash:"):
            digest = line.partition(":")[2].strip().casefold()
            if re.fullmatch(r"[0-9a-f]{64}", digest) is not None:
                current.digest = digest
    return states


def has_active_confirmed_image(output: str, digest: str) -> bool:
    return any(
        state.slot == 0
        and state.digest == digest
        and "active" in state.flags
        and "confirmed" in state.flags
        for state in parse_image_states(output)
    )


def has_secondary_image(output: str, digest: str) -> bool:
    return any(
        state.slot == 1 and state.digest == digest
        for state in parse_image_states(output)
    )


def run_mcumgr(
    executable: str,
    port: str,
    *arguments: str,
    expect_images: bool = False,
    timeout_seconds: int = 300,
    baud: int = DEFAULT_USB_CDC_BAUD,
    mtu: int = DEFAULT_SERIAL_MTU,
    measure_bytes: int | None = None,
) -> str:
    """实时转发 stdout，以独立 reader 避免管道阻塞，并在单调期限后终止子进程。"""
    command = mcumgr_command(executable, port, *arguments, baud=baud, mtu=mtu)
    print("+ " + shlex.join(command), flush=True)
    started = time.monotonic()
    chunks: list[bytes] = []
    try:
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
    except OSError as error:
        raise UploadError(f"unable to start mcumgr: {error}") from error

    assert process.stdout is not None
    output_queue: queue.Queue[bytes | None] = queue.Queue()
    reader_errors: list[OSError] = []

    def read_output() -> None:
        try:
            while True:
                chunk = os.read(process.stdout.fileno(), 4096)
                if not chunk:
                    break
                output_queue.put(chunk)
        except OSError as error:
            reader_errors.append(error)
        finally:
            output_queue.put(None)

    reader = threading.Thread(
        target=read_output,
        name="mcumgr-output",
        daemon=True,
    )
    deadline = started + timeout_seconds
    timed_out = False
    reader_done = False
    last_byte = b"\n"
    try:
        reader.start()
        while not reader_done:
            if time.monotonic() >= deadline:
                timed_out = True
                break
            try:
                chunk = output_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if chunk is None:
                reader_done = True
                continue
            chunks.append(chunk)
            last_byte = chunk[-1:]
            output_buffer = getattr(sys.stdout, "buffer", None)
            if output_buffer is not None:
                output_buffer.write(chunk)
                output_buffer.flush()
            else:
                sys.stdout.write(chunk.decode("utf-8", errors="replace"))
                sys.stdout.flush()
        if timed_out:
            raise UploadError(
                "mcumgr command timed out: " + " ".join(arguments)
            )
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired as error:
            raise UploadError(
                "mcumgr closed its output but did not exit within 5s"
            ) from error
        reader.join(timeout=1.0)
        if chunks and last_byte != b"\n":
            print()

        if reader_errors:
            raise UploadError(f"unable to read mcumgr output: {reader_errors[0]}")

        output = decode_output(b"".join(chunks))
        if process.returncode != 0:
            raise UploadError(
                f"mcumgr command failed with exit status {process.returncode}: "
                + " ".join(arguments)
            )
        device_error = protocol_error(output)
        if device_error is not None:
            raise UploadError(
                f"MCUboot rejected {' '.join(arguments)} ({device_error})"
            )
        if expect_images and MCUMGR_IMAGES_RE.search(output) is None:
            raise UploadError("mcumgr response returned no Images section")
        if measure_bytes is not None:
            elapsed = time.monotonic() - started
            # 速率 = 镜像字节数 / 实际墙钟秒 / 1024，仅作传输诊断，不作为成功条件。
            rate = measure_bytes / max(elapsed, 0.001) / 1024.0
            print(
                f"Transferred {measure_bytes} bytes in {elapsed:.2f}s "
                f"({rate:.2f} KiB/s)",
                flush=True,
            )
        return output
    finally:
        # reader 启动、输出转发、超时、Ctrl+C、返回码检查任一路径退出时，都先回收
        # 本函数拥有的 mcumgr；不得让它脱离 make/python 后继续独占 Windows COM。
        if process.poll() is None or (
            reader.ident is not None and reader.is_alive()
        ):
            _stop_owned_mcumgr(process, reader)
        if process.stdout is not None:
            process.stdout.close()
