"""STM32 定时器级联、互补输出极性与 CubeMX 零比较值 Safe-Off 门禁。"""

from __future__ import annotations

import re

from architecture.common import (
    NONZERO_PWM_PULSE_RE,
    ROOT,
    Violation,
    line_for,
    require_literals,
)


def scan_timer_contract(violations: list[Violation]) -> None:
    """同时核对生成 C 源与 .ioc 权威配置，避免重新生成后悄然丢失安全波形合同。"""
    timer_source = ROOT / "Core/Src/tim.c"
    require_literals(
        timer_source,
        (
            ("sConfigOC.Pulse = 0;", "R123",
             "CubeMX PWM compare defaults must remain zero"),
            ("sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;", "R127",
             "TIM5 must remain a reset-mode slave"),
            ("sSlaveConfig.InputTrigger = TIM_TS_ITR3;", "R128",
             "TIM5 must remain connected to TIM8 TRGO"),
            ("sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;", "R129",
             "TIM8 must publish its update event as TRGO"),
            ("sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;", "R138",
             "TIM8 complementary outputs must map zero compare to low"),
        ),
        violations,
    )
    if timer_source.is_file():
        text = timer_source.read_text(encoding="utf-8")
        for match in re.finditer(r"sConfigOC\.Pulse\s*=\s*([^;]+);", text):
            if match.group(1).strip() not in {"0", "0U", "0UL"}:
                violations.append(Violation(
                    timer_source, line_for(text, match.group(0)), "R124",
                    "CubeMX timer init writes a non-zero compare",
                ))

    ioc = ROOT / "H743_FreeRTOS.ioc"
    require_literals(
        ioc,
        (
            ("TIM5.TIM_SlaveMode=TIM_SLAVEMODE_RESET", "R139",
             "CubeMX TIM5 reset-slave contract is missing"),
            ("VP_TIM5_VS_ClockSourceITR.Mode=TriggerSource_ITR3", "R157",
             "CubeMX TIM5 ITR3 virtual connection is missing"),
            ("TIM8.OCNPolarity_2=TIM_OCNPOLARITY_LOW", "R158",
             "CubeMX TIM8 CH2N polarity is not safe at zero compare"),
            ("TIM8.OCNPolarity_3=TIM_OCNPOLARITY_LOW", "R159",
             "CubeMX TIM8 CH3N polarity is not safe at zero compare"),
            ("TIM8.TIM_MasterOutputTrigger=TIM_TRGO_UPDATE", "R160",
             "CubeMX TIM8 update TRGO contract is missing"),
        ),
        violations,
    )
    if ioc.is_file():
        for line_number, line in enumerate(
                ioc.read_text(encoding="utf-8").splitlines(), 1):
            if NONZERO_PWM_PULSE_RE.match(line):
                violations.append(Violation(
                    ioc, line_number, "R125",
                    "CubeMX PWM pulse default must remain zero",
                ))
