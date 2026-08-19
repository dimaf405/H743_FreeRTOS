"""Stable command-line surface for Make progress reporting."""

from __future__ import annotations

import argparse

from .memory import summary
from .models import PLAN_TOKEN
from .runner import run_step
from .state import finish, prepare

def parser() -> argparse.ArgumentParser:
    main_parser = argparse.ArgumentParser()
    subparsers = main_parser.add_subparsers(dest="operation", required=True)

    prepare_parser = subparsers.add_parser("prepare")
    prepare_parser.add_argument("--plan", required=True)
    prepare_parser.add_argument("--state", required=True)
    prepare_parser.add_argument("--goals", required=True)
    prepare_parser.add_argument("--no-color", action="store_true")
    prepare_parser.set_defaults(handler=prepare)

    run_parser = subparsers.add_parser("run")
    run_parser.add_argument("--state", default="")
    run_parser.add_argument("--plan-token", default=PLAN_TOKEN)
    run_parser.add_argument("--kind")
    run_parser.add_argument("--label")
    run_parser.add_argument("--target", required=True)
    run_parser.add_argument("--source")
    run_parser.add_argument("--display")
    run_parser.add_argument("--verbose", action="store_true")
    run_parser.add_argument("--quiet-command", action="store_true")
    run_parser.add_argument("--no-color", action="store_true")
    run_parser.add_argument("command", nargs=argparse.REMAINDER)
    run_parser.set_defaults(handler=run_step)

    finish_parser = subparsers.add_parser("finish")
    finish_parser.add_argument("--state", required=True)
    finish_parser.add_argument("--no-color", action="store_true")
    finish_parser.set_defaults(handler=finish)

    summary_parser = subparsers.add_parser("summary")
    summary_parser.add_argument("--goals", required=True)
    summary_parser.add_argument("--version", required=True)
    summary_parser.add_argument("--app-elf", required=True)
    summary_parser.add_argument("--boot-bin", required=True)
    summary_parser.add_argument("--signed", required=True)
    summary_parser.add_argument("--factory", required=True)
    summary_parser.add_argument("--layout", required=True)
    summary_parser.add_argument("--objdump", default="")
    summary_parser.add_argument("--size-tool", default="")
    summary_parser.add_argument("--no-color", action="store_true")
    summary_parser.set_defaults(handler=summary)

    return main_parser


def main() -> int:
    arguments = parser().parse_args()
    return int(arguments.handler(arguments))

