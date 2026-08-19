#!/usr/bin/env python3
"""Stable CLI entry for the modular MCUboot USB uploader."""

from __future__ import annotations

import sys

from upload.cli import main
from upload.models import UploadError


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except UploadError as error:
        print(f"upload failed: {error}", file=sys.stderr)
        raise SystemExit(1)
    except KeyboardInterrupt:
        print("upload cancelled", file=sys.stderr)
        raise SystemExit(130)
