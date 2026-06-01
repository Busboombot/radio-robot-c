#!/usr/bin/env python3

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

import requests
from dotenv import load_dotenv
import os

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lib import device_link  # noqa: E402


ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Deploy a built micro:bit hex file")
    parser.add_argument("--hex", dest="hex_path", default=None, help="Hex path (default auto-detect)")
    parser.add_argument("--console-url", default=None, help="Console base URL")
    parser.add_argument("--console-key", default=None, help="Console auth key")
    parser.add_argument("--usb-mount", default=None, help="USB mount path (default: auto-detect the robot)")
    parser.add_argument("--timeout", type=int, default=30, help="HTTP timeout seconds")
    parser.add_argument(
        "--force",
        action="store_true",
        help="Flash a volume even when its device type can't be confirmed. "
        "Never overrides the radio-relay safety check.",
    )
    return parser.parse_args()


def resolve_hex_path(explicit_hex: str | None) -> Path:
    if explicit_hex:
        path = Path(explicit_hex).expanduser().resolve()
        if not path.exists():
            raise FileNotFoundError(f"Hex file not found: {path}")
        return path

    candidates = [
        ROOT / "MICROBIT.hex",
        ROOT / "build" / "MICROBIT.hex",
        ROOT / "built" / "binary.hex",
    ]

    for path in candidates:
        if path.exists():
            return path

    raise FileNotFoundError(
        "No hex file found. Expected one of: MICROBIT.hex, build/MICROBIT.hex, built/binary.hex"
    )


def deploy_console(console_url: str, console_key: str, hex_path: Path, timeout: int) -> None:
    endpoint = f"{console_url.rstrip('/')}/api/hex"
    data = hex_path.read_bytes()

    response = requests.post(
        endpoint,
        data=data,
        headers={
            "Authorization": console_key,
            "Content-Type": "application/octet-stream",
        },
        timeout=timeout,
    )

    if response.status_code < 200 or response.status_code >= 300:
        raise RuntimeError(f"Console returned HTTP {response.status_code}: {response.text}")

    print(f"Console responded: HTTP {response.status_code}")
    if response.text:
        print(response.text)


class RelayProtectionError(RuntimeError):
    """Raised when the resolved flash target is (or might be) the radio relay."""


def _describe(dev: device_link.Device) -> str:
    role = dev.role or "unknown type"
    name = f" '{dev.common_name}'" if dev.common_name else ""
    return f"{role}{name} (serial {dev.serial or '?'})"


def _guard_target(dev: device_link.Device, force: bool) -> None:
    """Refuse to flash the relay; refuse unknown types unless --force."""
    if dev.is_relay:
        # The radio relay must never be overwritten — no --force escape hatch.
        raise RelayProtectionError(
            f"Refusing to flash {dev.volume}: it is the RADIO RELAY "
            f"[{_describe(dev)}]. Connect the robot, or pick the robot's volume "
            f"with --usb-mount. See docs/DEVICE_LINKING.md."
        )
    if not dev.type_known:
        if not force:
            raise RelayProtectionError(
                f"Could not confirm the device type at {dev.volume} "
                f"(serial {dev.serial or '?'}, port {dev.port or 'unlinked'}). "
                "Free the serial port (close any monitor/rogo session) and retry, "
                "or pass --force to flash anyway. Refusing by default so the "
                "radio relay is never overwritten."
            )
        print(
            f"WARNING: device type at {dev.volume} unconfirmed; flashing anyway (--force)."
        )


def resolve_usb_target(usb_mount: str | None, force: bool) -> Path:
    """Pick the volume to flash, proving it is not the radio relay."""
    explicit = usb_mount or os.environ.get("MICROBIT_MOUNT")
    if explicit:
        mount_path = Path(explicit).expanduser()
        if not mount_path.exists() or not mount_path.is_dir():
            raise FileNotFoundError(
                f"USB mount not found: {mount_path}. Connect micro:bit and verify "
                "mount path or set --usb-mount"
            )
        dev = device_link.classify_volume(mount_path)
        _guard_target(dev, force)
        print(f"Target: {mount_path}  [{_describe(dev)}]")
        return mount_path

    # Auto-detect: enumerate every micro:bit and pick the robot.
    devices = device_link.enumerate_devices(announce=True)
    if not devices:
        raise FileNotFoundError(
            "No micro:bit volume found. Connect the robot, or pass --usb-mount."
        )

    robots = [d for d in devices if d.is_robot]
    relays = [d for d in devices if d.is_relay]
    unknown = [d for d in devices if not d.type_known]

    if len(robots) == 1:
        dev = robots[0]
        if relays:
            print(f"Skipping radio relay: {_describe(relays[0])}")
        print(f"Target: {dev.volume}  [{_describe(dev)}]")
        return dev.volume

    if len(robots) > 1:
        listing = ", ".join(f"{d.volume} [{_describe(d)}]" for d in robots)
        raise RelayProtectionError(
            f"Multiple robot volumes found: {listing}. Specify one with --usb-mount."
        )

    # No positively-identified robot. Refuse rather than guess.
    seen = ", ".join(f"{d.volume} [{_describe(d)}]" for d in devices)
    if unknown and force:
        if len(unknown) == 1 and not relays:
            dev = unknown[0]
            print(
                f"WARNING: no confirmed robot; flashing the only unknown device "
                f"{dev.volume} [{_describe(dev)}] (--force)."
            )
            return dev.volume
        raise RelayProtectionError(
            f"--force given but the target is ambiguous (saw: {seen}). "
            "Pick one explicitly with --usb-mount."
        )
    raise RelayProtectionError(
        f"No confirmed robot to flash (saw: {seen}). Connect the robot and free "
        "the serial port, or pass --usb-mount (with --force if the type can't be "
        "read). Refusing by default so the radio relay is never overwritten."
    )


def deploy_usb(hex_path: Path, usb_mount: str | None, force: bool = False) -> None:
    mount_path = resolve_usb_target(usb_mount, force)
    destination = mount_path / hex_path.name
    shutil.copy2(hex_path, destination)
    print(f"Copied {hex_path} -> {destination}")


def main() -> int:
    args = parse_args()

    load_dotenv(ROOT / ".env")

    console_url = args.console_url or os.environ.get("CONSOLE_URL")
    console_key = args.console_key or os.environ.get("CONSOLE_KEY")

    try:
        hex_path = resolve_hex_path(args.hex_path)
    except FileNotFoundError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    if console_url and console_key:
        print(f"Deploying via console: {console_url}")
        try:
            deploy_console(console_url, console_key, hex_path, args.timeout)
            print("Deploy path: console (HTTP POST)")
            return 0
        except Exception as exc:
            print(f"Console deploy failed: {exc}", file=sys.stderr)
            return 1

    print("CONSOLE_URL and/or CONSOLE_KEY not set. Using local USB deploy.")
    try:
        deploy_usb(hex_path, args.usb_mount, args.force)
        print("Deploy path: local USB copy")
        return 0
    except Exception as exc:
        print(f"Local deploy failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
