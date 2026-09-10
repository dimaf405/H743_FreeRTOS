# Mission 任务存储

`SYS_DM_BACKEND` 已从权威 YAML 移除，由正式工具重新生成参数目录，不提供别名或虚拟参数。
任务只使用 SD 或 RAM；模块不持有 FlashFS，不读取、写入或擦除旧任务 Flash 记录。

## SD 与 RAM 行为

- 启动时尝试从 `0:/dima/mission.bin` 恢复完整任务；无可用 SD 或没有有效任务文件时，
  RAM 仓库为空，仍可通过 MAVLink 上传和执行任务。
- 有可用 SD 时使用现有 `AtomicFileDomain::Mission` 的 primary/backup/tmp 原子文件事务，
  完成写入、同步、回读校验和改名后才发送成功 commit result。
- 无卡时直接提交到 RAM，不写任务正文、版本标记或任何任务 Flash 副本。日志明确标注
  `RAM (volatile)`，成功 ACK 表示车辆已接收任务，不代表掉电保存。
- SD 初始化/写入失败后，只有本实例借用的文件事务终止并归还缓冲，才允许回退到 RAM。
  `EBUSY/EDEADLK/EAGAIN` 等待，`ESTALE` 先重新 load/validate，不误判成无卡。
- 无卡期间的上传、清空和入口调整都不持久化；重启后 RAM 状态清空，也不会改变已拔出
  SD 上的旧文件。之后带卡启动仍以该卡最后成功保存的任务为准。

## 热插卡、执行与停止

未解锁且空闲时约每 3 s 检查 SD；空间不足时退避至 60 s。启动后尚未接收过 RAM 任务，
可在插卡后恢复卡中的有效任务。已经上传或清空过的 RAM 快照在本次运行期优先，不能被旧卡任务替换；
SD 恢复后把当前 RAM 任务写入 SD，包括明确的空任务。

写回持有 maintenance interlock，不改变 AUTO 已推进的运行 current，不产生用户请求的 ACK。
文件 current 只保存上传/设置入口确定的逻辑入口。Armed 期间不启动后台迁移，执行继续使用 RAM 快照。
暂停、取消和关闭的 SD I/O 由 `wq:storage` 执行；调用方等借出的缓冲归还后再释放模块资源。

新 SD 快照是固定小端封装：版本、64 位文件代次、持久 current、载荷长度、标准 MAVLink Mission
帧及覆盖头/载荷的 CRC32。航点帧与任务内容 ID 复用原 `MissionCodec`，没有私有 MAVLink 消息或
手写消息列表。文件代次不回绕；空任务保留合法的 `MISSION_COUNT(count=0)`，不以删除文件代表 Clear。

## 日志核对

任务存储与 ULog 是不同链路。本次保留 `SDLOG_MODE`、`SDLOG_PROFILE` 和 `SDLOG_DIRS_MAX`。
默认 `SDLOG_MODE=0` 仅在 Armed 区间记录，因此“有卡就始终记录”不是当前默认日志行为；
`SDLOG_MODE=2` 才从启动记录到关闭。无可用 SD 时不写 ULog，也不会把 ULog 转写到 Flash；
记录意图仍存在时，重新插卡按原有重试/空间保护流程创建新日志会话。

上述内容是源码合同。实际拔插、SD 写入故障、掉电与 ACK 的板端行为需另行验证。
