# Dima 上游源码与许可证清单

- 日期：2026-07-29
- 文档状态：阶段 0 基线
- 许可证决策：`PENDING`

## 1. 管理规则

本清单用于记录 Dima Rover 所复用、移植或参考的外部飞控源码。目录改为 Dima 命名不代表来源发生变化，也不得用于隐藏或改变原始许可证义务。

从第一次导入开始必须遵守：

1. 保留上游文件原始版权头和许可证声明。
2. 记录上游仓库、tag/branch、完整 commit、原始路径和本地对应路径。
3. 记录本地修改摘要，不把移植代码描述为完全自研。
4. 不对上游 namespace、宏、参数名和 Topic 名进行品牌式全文替换。
5. 许可证状态为 `PENDING` 时，代码可用于内部技术评估和适配，但不得默认认定适合闭源二进制对外分发。
6. 对外发布前必须完成许可证审查，并确认源码提供、Notice、许可证全文和分发流程满足实际义务。
7. 本清单只记录来源，不替代正式法律意见。

## 2. PX4 正式移植基线

| 字段 | 内容 |
|---|---|
| 用途 | Parameter、ModuleParams、uORB API/消息契约、WorkQueue 接口、SBUS、RCUpdate、ManualControl、Commander Rover 子集、RoverDifferential、执行器链和 EKF2 |
| 正式目标版本 | PX4 v1.17.0 |
| 正式 commit | 待取得 v1.17.0 源码后填写 |
| 当前状态 | `PLANNED`，阶段 0 不导入生产源码 |
| 许可证状态 | `PENDING`；逐文件保留原始许可证 |
| 本地目录规则 | 产品目录使用 Dima；上游符号和许可证文字保持原样 |

正式导入前必须验证 tag/commit，并将每个导入文件登记到“文件级映射”表。

## 3. PX4 本地预研快照

| 字段 | 内容 |
|---|---|
| 本地分支 | `release/1.16` |
| 本地 commit（短） | `75f9a32a12` |
| 用途 | 阶段 0 架构预研、模块依赖和适配边界分析 |
| 限制 | 不能称为 v1.17.0，不能代替正式基线验证 |
| 导入状态 | 阶段 0 不从该快照导入生产源码 |

如后续需要记录完整 commit，应从本地仓库重新读取并补充，不根据短 commit 推测。

## 4. ArduPilot Rover 行为参考基线

| 字段 | 内容 |
|---|---|
| 参考 commit | `3f2e4763accb` |
| 用途 | Arming、Failsafe、RC 行为、轮速、差速车导航、倒车和 PivotTurn 行为参考 |
| 直接代码来源 | 默认否；未经许可证决策不得直接复制 GPL 实现到可对外发布产品 |
| EKF3 状态 | 不采用；此前 EKF3 计划已由 EKF2 最终选择取代 |
| 许可证状态 | `PENDING` |

ArduPilot 参考结论可以用于确定功能需求、状态机和验收行为；若未来直接移植文件，必须单独登记原始路径、版权头和 GPL 义务。

## 5. 计划中的上游模块映射

| 子系统 | 上游来源 | 计划本地位置 | 阶段 | 当前状态 |
|---|---|---|---:|---|
| 时间、WorkQueue、uORB 兼容接口 | PX4 v1.17.0 | `Dima/platform/freertos/`、`Dima/middleware/` | 1 | PLANNED |
| Parameter、ModuleParams | PX4 v1.17.0 | `Dima/middleware/parameters/` | 2 | PLANNED |
| SBUS、SbusRc、RCUpdate、ManualControl | PX4 v1.17.0 | `Dima/modules/rc/` | 3 | PLANNED |
| Commander Rover 子集 | PX4 v1.17.0；APM 行为参考 | `Dima/modules/safety/` | 4 | PLANNED |
| RoverDifferential 与执行器链 | PX4 v1.17.0；APM 行为参考 | `Dima/modules/rover/` | 5 | PLANNED |
| EKF2 与 Estimator 支撑库 | PX4 v1.17.0 | `Dima/modules/estimator/ekf2/`、`Dima/lib/estimator/` | 7 | PLANNED |
| Position、Waypoint、Reverse、PivotTurn | PX4 v1.17.0；APM 行为参考 | `Dima/modules/rover/` | 9 | PLANNED |

## 6. 文件级映射

阶段 0 不导入上游生产源码，因此当前没有文件级条目。后续每次导入必须按以下格式追加，不得覆盖历史记录：

| 上游仓库 | tag/commit | 原始路径 | 本地路径 | 许可证 | 本地修改摘要 | 导入日期 |
|---|---|---|---|---|---|---|
| 待填写 | 待填写 | 待填写 | 待填写 | 待填写 | 待填写 | YYYY-MM-DD |

## 7. 发布限制

在许可证状态从 `PENDING` 变更为已审查之前：

- 不得把包含许可证未决上游代码的固件标记为可公开发布版本。
- 不得删除、缩短或改写上游版权头和许可证声明。
- 不得因为目录、namespace 包装或接口适配而宣称代码来源已经改变。
- 内部构建产物必须标记为研发或评估用途。
- 发布评审必须核对本清单、实际编译文件、Notice、许可证全文和源码提供方式。
