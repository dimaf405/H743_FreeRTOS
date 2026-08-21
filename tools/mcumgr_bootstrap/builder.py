"""Pinned Apache mcumgr source adaptation and binary provisioning."""

from __future__ import annotations

import hashlib
import json
import os
import pathlib
import shutil
import subprocess
import tempfile

from .errors import BootstrapError
from .go_toolchain import host_platform, provision_go


MCUMGR_MODULE = "github.com/apache/mynewt-mcumgr-cli"
MCUMGR_PACKAGE = "./mcumgr"
MCUMGR_VERSION = "v0.0.0-20221004073047-5c56bd24066c"
NEWTMGR_MODULE = "mynewt.apache.org/newtmgr"
NEWTMGR_VERSION = "v0.0.0-20201028150837-60b2da78788c"
MCUMGR_BUILD_REVISION = "dima-usb-cdc-v2"
MCUMGR_VERSION_STRING = f"0.0.0-dev+{MCUMGR_BUILD_REVISION}"
DEFAULT_GOPROXY = "https://goproxy.cn,direct"

NEWTMGR_SERIAL_XPORT_SHA256 = (
    "1ea559513ae11a658bc571cc25793995e1528130954a7dba6e7a97a4d050e942"
)
NEWTMGR_IMAGE_XACT_SHA256 = (
    "fea8b780eff391207b6c1b4f7cc90370271d4bc87b98d3b2275232fdd860ce42"
)
NEWTMGR_SERIAL_DELAY = "\t\t\ttime.Sleep(20 * time.Millisecond)\n"
NEWTMGR_USB_CDC_DELAY = (
    "\t\t\t// Real UART transports need pacing between 124-byte NLIP frames.\n"
    "\t\t\t// Dima Rover uses USB CDC at a virtual baud of 921600 or higher,\n"
    "\t\t\t// where this delay only throttles firmware upload throughput.\n"
    "\t\t\tif sx.cfg.Baud < 921600 {\n"
    "\t\t\t\ttime.Sleep(20 * time.Millisecond)\n"
    "\t\t\t}\n"
)
NEWTMGR_FIXED_FRAME = "\t\twriteLen := util.Min(124, totlen-written)\n"
NEWTMGR_CONFIGURED_FRAME = (
    "\t\twriteLen := util.Min(sx.cfg.Mtu-4, totlen-written)\n"
)
NEWTMGR_UNALIGNED_CHUNK_RETURN = "\treturn chunklen, nil\n"
NEWTMGR_ALIGNED_CHUNK_RETURN = (
    "\t// STM32H743 flash words are 32 bytes.  Align every non-final chunk so\n"
    "\t// MCUboot acknowledges exactly the offset that the upload window sent.\n"
    "\tif chunklen < len(data)-off {\n"
    "\t\tchunklen -= chunklen % 32\n"
    "\t}\n\n"
    "\treturn chunklen, nil\n"
)


def mcumgr_works(executable: pathlib.Path) -> bool:
    try:
        completed = subprocess.run(
            [str(executable), "version"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=10,
            check=False,
            text=True,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return (
        completed.returncode == 0
        and completed.stdout.strip() == f"mcumgr {MCUMGR_VERSION_STRING}"
    )


def go_module_directory(
    go: pathlib.Path, environment: dict[str, str], module: str, version: str
) -> pathlib.Path:
    try:
        completed = subprocess.run(
            [str(go), "mod", "download", "-json", f"{module}@{version}"],
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=300,
            check=False,
            text=True,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise BootstrapError(
            f"unable to download Go module {module}: {error}"
        ) from error

    if completed.returncode != 0:
        details = (completed.stderr or completed.stdout).strip()
        raise BootstrapError(
            f"unable to download Go module {module}@{version}: {details}"
        )
    try:
        metadata = json.loads(completed.stdout)
        directory = pathlib.Path(metadata["Dir"])
    except (KeyError, TypeError, json.JSONDecodeError) as error:
        raise BootstrapError(
            f"Go returned malformed module metadata for {module}@{version}"
        ) from error
    if not directory.is_dir():
        raise BootstrapError(f"downloaded Go module directory is missing: {directory}")
    return directory


def replace_once(path: pathlib.Path, old: str, new: str, description: str) -> None:
    contents = path.read_text(encoding="utf-8")
    if contents.count(old) != 1:
        raise BootstrapError(
            f"cannot apply {description}; expected source context was not unique "
            f"in {path}"
        )
    path.write_text(contents.replace(old, new), encoding="utf-8")


def prepare_mcumgr_sources(
    go: pathlib.Path, environment: dict[str, str], destination: pathlib.Path
) -> pathlib.Path:
    mcumgr_source = go_module_directory(
        go, environment, MCUMGR_MODULE, MCUMGR_VERSION
    )
    newtmgr_source = go_module_directory(
        go, environment, NEWTMGR_MODULE, NEWTMGR_VERSION
    )

    mcumgr_build = destination / "mcumgr"
    newtmgr_build = destination / "newtmgr"
    shutil.copytree(mcumgr_source, mcumgr_build, copy_function=shutil.copyfile)
    shutil.copytree(newtmgr_source, newtmgr_build, copy_function=shutil.copyfile)

    serial_xport = newtmgr_build / "nmxact" / "nmserial" / "serial_xport.go"
    digest = hashlib.sha256(serial_xport.read_bytes()).hexdigest()
    if digest != NEWTMGR_SERIAL_XPORT_SHA256:
        raise BootstrapError(
            "Apache newtmgr serial source failed its pinned SHA-256 check: "
            f"expected {NEWTMGR_SERIAL_XPORT_SHA256}, got {digest}"
        )
    replace_once(
        serial_xport,
        NEWTMGR_SERIAL_DELAY,
        NEWTMGR_USB_CDC_DELAY,
        "Dima USB CDC pacing patch",
    )
    replace_once(
        serial_xport,
        NEWTMGR_FIXED_FRAME,
        NEWTMGR_CONFIGURED_FRAME,
        "configurable serial-frame patch",
    )

    image_xact = newtmgr_build / "nmxact" / "xact" / "image.go"
    digest = hashlib.sha256(image_xact.read_bytes()).hexdigest()
    if digest != NEWTMGR_IMAGE_XACT_SHA256:
        raise BootstrapError(
            "Apache newtmgr image-upload source failed its pinned SHA-256 check: "
            f"expected {NEWTMGR_IMAGE_XACT_SHA256}, got {digest}"
        )
    replace_once(
        image_xact,
        NEWTMGR_UNALIGNED_CHUNK_RETURN,
        NEWTMGR_ALIGNED_CHUNK_RETURN,
        "STM32H743 upload-chunk alignment patch",
    )

    replace_once(
        mcumgr_build / "mcumgr" / "mcumgr.go",
        'VersionString: "0.0.0-dev",',
        f'VersionString: "{MCUMGR_VERSION_STRING}",',
        "mcumgr build identity patch",
    )
    go_mod = mcumgr_build / "go.mod"
    with go_mod.open("a", encoding="utf-8") as output:
        output.write(f"\nreplace {NEWTMGR_MODULE} => ../newtmgr\n")
    return mcumgr_build


def ensure_mcumgr(
    cache_root: pathlib.Path,
    target_windows: bool,
    announce_cache_hit: bool = True,
) -> pathlib.Path:
    host_os, host_arch = host_platform()
    target_os = "windows" if target_windows else host_os
    executable_name = "mcumgr.exe" if target_os == "windows" else "mcumgr"
    destination = (
        cache_root
        / "mcumgr"
        / f"{MCUMGR_VERSION.removeprefix('v')}-{MCUMGR_BUILD_REVISION}"
        / f"{target_os}-{host_arch}"
        / executable_name
    )
    if destination.is_file() and mcumgr_works(destination):
        if announce_cache_hit:
            print(f"Using cached Apache mcumgr: {destination}", flush=True)
        return destination

    go = provision_go(cache_root, host_os, host_arch)
    workspace = cache_root / "go-workspace"
    workspace.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    environment.pop("GOBIN", None)
    environment.update(
        {
            "CGO_ENABLED": "0",
            "GOARCH": host_arch,
            "GOOS": target_os,
            "GOPATH": str(workspace),
            "GOTOOLCHAIN": "local",
            "GOPROXY": environment.get("DIMA_GOPROXY")
            or environment.get("GOPROXY")
            or DEFAULT_GOPROXY,
        }
    )

    print(
        f"Building pinned Apache mcumgr {MCUMGR_VERSION} "
        f"({MCUMGR_BUILD_REVISION}) for {target_os}/{host_arch} ...",
        flush=True,
    )
    cache_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix=".mcumgr-build-", dir=cache_root
    ) as temporary_name:
        build_root = pathlib.Path(temporary_name)
        source_root = prepare_mcumgr_sources(go, environment, build_root)
        built_binary = build_root / executable_name
        try:
            completed = subprocess.run(
                [
                    str(go),
                    "build",
                    "-trimpath",
                    "-o",
                    str(built_binary),
                    MCUMGR_PACKAGE,
                ],
                cwd=source_root,
                env=environment,
                timeout=600,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            raise BootstrapError(f"unable to build Apache mcumgr: {error}") from error
        if completed.returncode != 0:
            raise BootstrapError(
                f"Apache mcumgr build failed with exit status {completed.returncode}"
            )
        if not built_binary.is_file():
            raise BootstrapError(
                f"Go did not produce the expected binary: {built_binary}"
            )

        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary_destination = destination.with_name(
            f".{destination.name}.tmp.{os.getpid()}"
        )
        try:
            shutil.copy2(built_binary, temporary_destination)
            temporary_destination.chmod(0o755)
            os.replace(temporary_destination, destination)
        finally:
            temporary_destination.unlink(missing_ok=True)

    if not mcumgr_works(destination):
        raise BootstrapError("the cached Apache mcumgr failed its version check")
    print(f"Cached Apache mcumgr: {destination}", flush=True)
    return destination
