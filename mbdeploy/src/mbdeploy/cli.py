"""mbdeploy CLI — entry point and subcommand definitions."""

from __future__ import annotations

import argparse
import sys


def _cmd_build(args: argparse.Namespace) -> int:
    print("build: not implemented")
    return 0


def _cmd_deploy(args: argparse.Namespace) -> int:
    print("deploy: not implemented")
    return 0


def _cmd_list(args: argparse.Namespace) -> int:
    print("list: not implemented")
    return 0


def _cmd_probe(args: argparse.Namespace) -> int:
    print("probe: not implemented")
    return 0


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="mbdeploy",
        description="Build and deploy micro:bit firmware to one or more devices.",
    )
    subparsers = parser.add_subparsers(dest="subcommand", metavar="<subcommand>")
    subparsers.required = True

    # --- build ---
    build_p = subparsers.add_parser(
        "build",
        help="Compile the micro:bit firmware.",
    )
    build_p.add_argument("--clean", action="store_true", help="Clean before building.")
    build_p.add_argument("--verbose", action="store_true", help="Show build output.")
    build_p.add_argument("-j", dest="jobs", type=int, metavar="N", help="Parallel jobs.")
    build_p.add_argument(
        "--build-cmd", metavar="CMD", help="Override the build command."
    )
    build_p.set_defaults(func=_cmd_build)

    # --- deploy ---
    deploy_p = subparsers.add_parser(
        "deploy",
        help="Flash firmware to one or more micro:bit devices.",
    )
    deploy_p.add_argument(
        "target",
        nargs="?",
        metavar="target",
        help="Target device serial number or alias (default: all).",
    )
    deploy_p.add_argument(
        "--build", action="store_true", help="Build before deploying."
    )
    deploy_p.add_argument(
        "--clean", action="store_true", help="Clean before building (implies --build)."
    )
    deploy_p.add_argument("-j", dest="jobs", type=int, metavar="N", help="Parallel jobs.")
    deploy_p.add_argument(
        "--force-relay",
        action="store_true",
        help="Force relay (USB hub power cycle) before flashing.",
    )
    deploy_p.add_argument("--hex", metavar="PATH", help="Path to a pre-built .hex file.")
    deploy_p.add_argument(
        "--target-mcu",
        metavar="MCU",
        default="nrf52833",
        help="Target MCU type (default: nrf52833).",
    )
    deploy_p.add_argument(
        "--config", metavar="PATH", help="Path to device config file."
    )
    deploy_p.set_defaults(func=_cmd_deploy)

    # --- list ---
    list_p = subparsers.add_parser(
        "list",
        help="List detected micro:bit devices.",
    )
    list_p.add_argument("--config", metavar="PATH", help="Path to device config file.")
    list_p.set_defaults(func=_cmd_list)

    # --- probe ---
    probe_p = subparsers.add_parser(
        "probe",
        help="Probe a micro:bit device and report its state.",
    )
    probe_p.add_argument("--config", metavar="PATH", help="Path to device config file.")
    probe_p.set_defaults(func=_cmd_probe)

    return parser


def main() -> None:
    parser = _build_parser()
    args = parser.parse_args()
    sys.exit(args.func(args))


if __name__ == "__main__":
    main()
