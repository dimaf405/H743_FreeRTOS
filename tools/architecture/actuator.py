"""执行器底层 PWM API 的硬件所有权门禁。"""

from __future__ import annotations

import re

from architecture.common import ROOT, Violation, first_party_sources


def scan_active_actuator_contract(violations: list[Violation]) -> None:
    """只限制裸板级/HAL PWM API，控制通道和状态机由产品需求自行演进。"""
    board_api_owners = {
        "Boards/H743/Inc/motor_pwm.h",
        "Boards/H743/Src/motor_pwm.c",
        "Dima/platform/stm32h7/pwm/ActuatorPwm.cpp",
    }
    for path in first_party_sources():
        relative = path.relative_to(ROOT).as_posix()
        for line_number, line in enumerate(
                path.read_text(encoding="utf-8").splitlines(), 1):
            board_call = re.search(
                r"\bboard_motor_pwm_(start|stop|write|started)\s*\(", line,
            )
            board_call_allowed = relative in board_api_owners or (
                relative == "Boards/H743/Src/board_init.c" and
                board_call is not None and board_call.group(1) == "stop"
            )
            if board_call is not None and not board_call_allowed:
                violations.append(Violation(
                    path, line_number, "R120",
                    "board motor PWM API is outside the board or STM32 "
                    "capability owner",
                ))
            if (relative != "Boards/H743/Src/motor_pwm.c" and
                    re.search(
                        r"\bHAL_TIM(?:Ex)?_PWMN?_(?:Start|Stop)\s*\(", line,
                    )):
                violations.append(Violation(
                    path, line_number, "R121",
                    "raw HAL PWM start/stop is outside the board motor backend",
                ))
