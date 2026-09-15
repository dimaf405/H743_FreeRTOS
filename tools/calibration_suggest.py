#!/usr/bin/env python3
"""只读分析 QGC [autocal] 消息及五列 .params（兼容 NAME VALUE 两列）。

只解释最近会话的实际证据，不写参数、不补默认值，也不复制固件参数/消息目录。
用法：python3 tools/calibration_suggest.py --messages messages.txt --params rover.params
退出码：0=存在终态，1=最近会话无终态，2=输入不存在或格式错误。
"""
from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path


def parse_messages(path: Path) -> dict:
    """按明确会话边界清理证据，同一会话的周期摘要和重复终态保持幂等。"""
    def empty(session_id=None) -> dict:
        return {"session_id": session_id, "final": None, "failure": None, "interference": None,
                "endpoint": False, "compensation": None, "unavailable": 0, "skipped": 0}

    result = empty()
    for match in re.finditer(r"\[autocal\][ \t]*([^\r\n]+)",
                             path.read_text(encoding="utf-8-sig")):
        line = match.group(1).strip()
        tagged = re.match(r"session=(\d+)\s+(.*)", line)
        if tagged:
            session_id, line = int(tagged.group(1)), tagged.group(2).strip()
            if result["session_id"] != session_id or line == "begin":
                result = empty(session_id)
        elif re.fullmatch(r"(?:STATE_)?PREFLIGHT_CHECK", line):
            # 兼容旧版单次入场行；周期的 PREFLIGHT_CHECK; ... 不是新会话。
            result = empty(result["session_id"])
        final = re.match(r"(SUCCESS|PARTIAL|FAILED|CANCELLED):\s*([A-Z0-9_]+)\b", line)
        if final:
            terminal = final.groups()
            if not tagged and result["session_id"] is None and result["final"] is not None and \
                    terminal != (result["final"], result["failure"]):
                # 旧日志只有不同终态、没有入场边界时，保守视为另一段，禁止继承。
                result = empty()
            result["final"], result["failure"] = terminal
        # 稳定的周期证据可恢复重连观察；提示中的 30% 阈值不是测量值。
        evidence = re.search(r"evidence endpoint=([01]) mag_pct=(-?\d+(?:\.\d+)?) mag_comp=([01]) mag_present=([01])", line)
        if evidence:
            endpoint, value, compensation, present = evidence.groups()
            value = float(value)
            result["endpoint"] = endpoint == "1"
            result["interference"] = value if math.isfinite(value) and value >= 0 else None
            result["compensation"] = compensation == "1" if present == "1" else None
        if "mag throttle" in line:
            measured = re.search(r"(-?\d+(?:\.\d+)?)%\s+interference", line)
            if measured:
                value = float(measured.group(1))
                result["interference"] = value if math.isfinite(value) and value >= 0 else None
            if "saved and applied" in line:
                result["compensation"] = True
            if "unobservable" in line:
                result["interference"] = None
            if "incomplete" in line or "unobservable" in line or "rolled back" in line:
                result["compensation"] = False
        if "command endpoint reached" in line:
            result["endpoint"] = True
        unavailable = re.search(r"unavailable=0x([0-9a-fA-F]+)", line)
        skipped = re.search(r"skipped=0x([0-9a-fA-F]+)", line)
        if unavailable:
            result["unavailable"] = int(unavailable.group(1), 16)
        if skipped:
            result["skipped"] = int(skipped.group(1), 16)
    return result


def parse_params(path: Path) -> dict:
    """QGC 五列为 system/component/name/value/type；含歧义或非法值时拒绝混读。"""
    values = {}
    owner = None
    for number, line in enumerate(path.read_text(encoding="utf-8-sig").splitlines(), 1):
        parts = line.partition("#")[0].split()
        if not parts:
            continue
        try:
            if len(parts) == 5:
                system, component = int(parts[0]), int(parts[1])
                wire_type = int(parts[4])
                if not (0 <= system <= 255 and 0 <= component <= 255 and 1 <= wire_type <= 10):
                    raise ValueError("设备标识或参数类型非法")
                current_owner = (system, component)
                if owner is not None and current_owner != owner:
                    raise ValueError("包含多个设备/组件，请导出单一目标的参数")
                owner = current_owner
                name, raw = parts[2:4]
            elif len(parts) == 2:
                name, raw = parts
                wire_type = None
            else:
                raise ValueError("应为 NAME VALUE 两列或 QGC 五列")
            value = float(raw)
            if not re.fullmatch(r"[A-Z][A-Z0-9_]{0,15}", name) or not math.isfinite(value):
                raise ValueError("参数名称或数值非法")
            if wire_type is not None and wire_type <= 8 and not value.is_integer():
                raise ValueError("整数参数带有小数")
            if name in values and values[name] != value:
                raise ValueError(f"参数 {name} 存在冲突值")
            values[name] = value
        except ValueError as error:
            raise ValueError(f"{path} 第 {number} 行：{error}") from error
    if not values:
        raise ValueError(f"{path} 没有可用参数")
    return values


def suggest(messages: dict, params: dict) -> list[str]:
    """以下是具体诊断规则，不枚举固件目录；缺失参数始终按未知处理。"""
    advice = []
    final, failure = messages.get("final"), messages.get("failure")
    if final is None:
        advice.append("最近会话未找到终态；请导出完整消息，不能据此认定校准成功。")
    if failure == "DRIVE_ENVELOPE":
        advice.append("驱动请求已到冻结包络，连续 8 秒仍无运动：核对 MOT_THR_MIN、供电、电池与机械传动；不要直接提高驱动上限。")
    elif failure in ("FENCE_SPACE", "FENCE_BOUNDARY", "STOP_DISTANCE"):
        advice.append("按当前载荷和地面实测可信停车距离，检查场地及 RO_CAL_RADIUS；不要缩小 RO_CAL_STOP_D 申报来通过围栏。")
    elif failure == "MOTION_UNAVAILABLE":
        advice.append("运动条件或控制余量不足：结合该会话前序消息核对输出安全链、速度配置及 FF 余量。")
    elif failure in ("RTK_QUALITY", "RTK_INCONSISTENT"):
        advice.append("检查双天线固定解、安装基线、航向与速度数据质量后重试。")
    elif failure == "MAG_DISTURBED":
        advice.append("磁校准未收敛：检查附近铁磁件及大电流线束布置。")
    elif failure in ("PROFILE_UNOBSERVABLE", "IDENTIFICATION", "GAIN_VALIDATION"):
        advice.append("响应或闭环证据未通过：结合前序失败阶段检查反馈、激励覆盖和 FF 余量；不凭离线文本猜测 PI 增益。")
    elif failure not in (None, "NONE", "OPERATOR_CANCEL"):
        advice.append(f"失败码 {failure} 暂无专属规则；保留原始消息并核对对应固件说明，不能视为正常。")
    interference = messages.get("interference")
    if interference is None:
        advice.append("磁干扰率未知：缺少有效回归结果，不能用未知代替 0%。")
    elif interference > 30.0:
        advice.append(f"磁干扰 {interference:.1f}% 超过 30%：优先整改磁力计、电机线束与大电流回路的布置。")
    if messages.get("compensation") is False:
        advice.append("磁-油门补偿未完成；基础磁偏置校准与其他已确认成果应分别核对。")
    if not messages.get("endpoint"):
        advice.append("本会话未找到有效顶档端点证据；不能据此推断电机最大转速或主动放宽速度/围栏。")
    if messages.get("skipped"):
        advice.append("固件明确跳过了能力范围外或无对应设备的项目；这些项目保持原值，没有被计作已完成。")
    if final == "SUCCESS":
        advice.append("SUCCESS 表示本次适用项目已验证并保存；跳过项目的能力仍未标定。")
    if final == "PARTIAL":
        advice.append("PARTIAL：保留已确认保存的阶段成果，按未完成项排查；该结果不表示全部参数已标定。")
    if not params:
        advice.append("未提供参数导出，参数配置未知；未使用固件默认值代填。")
    else:
        stop = params.get("RO_CAL_STOP_D")
        radius = params.get("RO_CAL_RADIUS")
        speed = params.get("RO_SPEED_LIM")
        envelope = params.get("MOT_THR_MAX")
        if any(value is None for value in (stop, radius, speed, envelope)):
            advice.append("参数导出缺少停车距离、围栏、速度或驱动包络信息，缺项按未知处理。")
        if stop is not None and stop <= 0:
            advice.append("RO_CAL_STOP_D 未给出正的可信停车距离，动态阶段不可用；按当前工况实测后填写。")
        if radius is not None and stop is not None and radius <= stop:
            advice.append("申报停车距离已不小于围栏半径，无法据此证明动态空间充足；核对场地和停车实测值。")
        if speed is not None and speed <= 0:
            advice.append("RO_SPEED_LIM 非正，本会话不允许动态校准。")
        if envelope is not None and not 0.05 <= envelope <= 1.0:
            advice.append("MOT_THR_MAX 超出当前固件支持域 [0.05, 1.0]，应核对配置。")
        identity = params.get("CAL_MAG_MOT_ID")
        generation = params.get("CAL_MAG_MOT_GEN")
        if identity == 0 or generation == 0:
            advice.append("导出快照的磁补偿 ID/GEN 为零，补偿无有效学习代；这些只读值应由固件校准更新。")
    if not advice:
        advice.append("文本中未发现需专门处置的异常，建议保留本会话消息、参数和 ULog。")
    return advice


def main() -> int:
    parser = argparse.ArgumentParser(description="自动校准离线建议（只读，不写参数）")
    parser.add_argument("--messages", required=True, type=Path, help="包含 [autocal] 行的消息文本")
    parser.add_argument("--params", type=Path, help="QGC 五列参数或 NAME VALUE 两列文本")
    arguments = parser.parse_args()
    try:
        messages = parse_messages(arguments.messages)
        params = parse_params(arguments.params) if arguments.params is not None else {}
    except (OSError, UnicodeError, ValueError) as error:
        print(f"输入错误：{error}", file=sys.stderr)
        return 2
    print(f"最近会话终态：{messages['final'] or '未知'}（失败码 {messages['failure'] or '未知'}）")
    interference = messages["interference"]
    print("磁干扰：未知" if interference is None else f"磁干扰：{interference:.1f}%")
    print(f"端点证据：{'已观测' if messages['endpoint'] else '未找到'}")
    for index, item in enumerate(suggest(messages, params), 1):
        print(f"{index}. {item}")
    return 0 if messages["final"] is not None else 1


if __name__ == "__main__":
    raise SystemExit(main())
