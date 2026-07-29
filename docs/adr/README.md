# Dima Rover 架构决策记录（ADR）

本目录记录对 Dima FreeRTOS Rover 有长期影响且需要追踪原因的架构决策。

## 使用规则

- 文件名格式：`NNNN-short-decision-name.md`。
- 日期使用 `YYYY-MM-DD`。
- 状态使用：`Proposed`、`Accepted`、`Superseded`、`Rejected`。
- 已接受 ADR 不覆盖历史；变更决策时新增 ADR，并在旧 ADR 中标注被取代关系。
- 与上游复用相关的决策必须同步更新 `../DIMA_SOURCE_MANIFEST.md`。
- 许可证状态未决时，ADR 只能记录工程选择，不能替代正式许可证审查。
- ADR 验证不通过新增测试或仿真框架完成；按对应阶段使用目标编译、链接检查、板上和实车验收。

## 当前 ADR

| 编号 | 标题 | 状态 | 日期 |
|---:|---|---|---|
| 0001 | 采用 FreeRTOS 构建 Dima 差速 Rover | Accepted | 2026-07-29 |
| 0002 | 优先复用上游模块并采用 Dima 目录命名 | Accepted | 2026-07-29 |
| 0003 | 采用 PX4 EKF2 作为最终状态估计器 | Accepted | 2026-07-29 |

## 相关文档

- `../DIMA_ROVER_PORTING_PLAN_ZH.md`
- `../DIMA_SOURCE_MANIFEST.md`
