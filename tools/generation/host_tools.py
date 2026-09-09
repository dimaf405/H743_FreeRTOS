#!/usr/bin/env python3
"""按内容身份安装生成/签名依赖；跨工作树共用 OS 文件锁与原子目录切换。"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import time
import uuid


def install(output: Path, signing: Path, generation: Path) -> None:
    # 该工具仅管理内容寻址缓存，拒绝把误传的仓库、用户目录作为替换目标。
    if output.parent.name != "host-python-envs" or re.fullmatch(r"[0-9a-f]{24}", output.name) is None:
        raise ValueError("output must be a content-addressed host-python-envs directory")
    output.parent.mkdir(parents=True, exist_ok=True)
    lock_path = output.parent / f".{output.name}.lock"
    with lock_path.open("a+b") as lock:
        lock.seek(0, os.SEEK_END)
        if lock.tell() == 0:
            lock.write(b"\0")
            lock.flush()
        # 锁仅保护同一内容身份的工具目录。内核在进程退出时释放锁，崩溃不留下
        # 永久 busy 标记；并发构建不能互相移走正在使用的 host-python。
        deadline = time.monotonic() + 180
        while True:
            try:
                lock.seek(0)
                if os.name == "nt":
                    import msvcrt
                    msvcrt.locking(lock.fileno(), msvcrt.LK_NBLCK, 1)
                else:
                    import fcntl
                    fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except OSError:
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"host-tools installation lock timed out: {lock_path}")
                time.sleep(0.2)
        try:
            if (output / ".installed").is_file():
                return
            with tempfile.TemporaryDirectory(prefix=f".{output.name}.stage-", dir=output.parent) as temporary:
                staging = Path(temporary) / "packages"
                command = [sys.executable, "-m", "pip", "install", "--disable-pip-version-check", "--target", str(staging)]
                subprocess.run([*command, "-r", str(signing)], check=True)
                subprocess.run([*command, "--upgrade", "--no-deps", "--require-hashes", "-r", str(generation)], check=True)
                (staging / ".installed").write_text(output.name + "\n", encoding="ascii")
                backup = output.with_name(f".{output.name}.old-{uuid.uuid4().hex}")
                previous = output.exists()
                if previous:
                    os.replace(output, backup)
                try:
                    os.replace(staging, output)
                except OSError:
                    if previous:
                        os.replace(backup, output)
                    raise
                if previous:
                    shutil.rmtree(backup)
        finally:
            lock.seek(0)
            if os.name == "nt":
                import msvcrt
                msvcrt.locking(lock.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                import fcntl
                fcntl.flock(lock.fileno(), fcntl.LOCK_UN)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--signing", type=Path, required=True)
    parser.add_argument("--generation", type=Path, required=True)
    arguments = parser.parse_args()
    try:
        install(arguments.output.resolve(), arguments.signing.resolve(), arguments.generation.resolve())
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f"host-tools: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
