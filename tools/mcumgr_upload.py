#!/usr/bin/env python3
"""模块化 MCUboot USB 上传器的稳定 CLI 入口；统一错误阶段和退出码。"""

from __future__ import annotations

import sys

from upload.cli import main
from upload.models import UploadError, stage


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except UploadError as error:
        stage("FAILED", f"upload failed: {error}", stream=sys.stderr)
        raise SystemExit(1)
    except KeyboardInterrupt:
        stage("CANCELLED", "upload cancelled", stream=sys.stderr)
        raise SystemExit(130)
