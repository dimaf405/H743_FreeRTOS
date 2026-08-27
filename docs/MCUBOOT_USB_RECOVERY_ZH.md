# STM32H743 MCUboot 升级与 USB 救援操作手册

本文适用于本工程的 STM32H743VI（LQFP100）固件。系统提供两条互相独立的 USB 恢复链路：

1. 正常升级和应用故障恢复：MCUboot USB CDC + `mcumgr`，只接收经过 ECDSA-P256 签名的镜像，并支持测试启动、自动确认和回滚。
2. MCUboot 本身损坏、时钟初始化失败或芯片无法运行用户 Flash 时的最终救援：芯片内部 ROM USB DFU + STM32CubeProgrammer。

ROM DFU 不依赖 MCUboot、FreeRTOS、应用固件或外部 HSE。要保证“死机仍可从同一个 USB 口救回”，量产硬件和 Option Bytes 必须满足本文的硬件检查项。

## 1. 当前 Flash 布局与构建产物

| 区域 | 地址范围 | 大小 | 用途 |
| --- | --- | ---: | --- |
| MCUboot | `0x08000000`–`0x0803FFFF` | 256 KiB | 独立 Bootloader |
| Primary slot | `0x08040000`–`0x080FFFFF` | 768 KiB | 当前运行镜像；MCUboot header 为 `0x400` 字节，应用向量表位于 `0x08040400` |
| Secondary slot | `0x08100000`–`0x081BFFFF` | 768 KiB | USB 上传的新镜像 |
| Scratch | `0x081C0000`–`0x081DFFFF` | 128 KiB | scratch swap 交换区 |
| 保留区 | `0x081E0000`–`0x081FFFFF` | 128 KiB | 保留，不用于本次升级 |

在工程根目录一条命令完成构建、签名与校验：

```bash
make dima_rover
```

如果交叉编译器不在 PATH，可追加
`GCC_PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin`，目标行为不变。

与烧写相关的产物为：

| 文件 | 用途 |
| --- | --- |
| `build/H743_FreeRTOS_signed.bin` | 通过 `mcumgr` 上传的签名升级包 |
| `build/H743_FreeRTOS_factory.hex` | STM32CubeProgrammer 工厂烧写/ROM DFU 救援包，包含 MCUboot 和已签名 Primary 镜像 |
| `build/mcuboot/mcuboot.hex` | 仅 MCUboot，通常不单独交付 |

不要将 `build/H743_FreeRTOS.bin`、未签名 HEX 或 `factory.hex` 传给 `mcumgr`。`factory.hex` 带有正确的 Flash 绝对地址，但有意不包含 Option Bytes。

## 2. 生产密钥

仓库内的 `.keys/development-ecdsa-p256.pem` 仅用于开发。生产构建必须显式覆盖 `KEY_FILE`，并保证该文件在构建前已经存在：

```bash
test -f /secure/path/production-ecdsa-p256.pem
make -j4 \
  GCC_PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin \
  KEY_FILE=/secure/path/production-ecdsa-p256.pem \
  IMAGE_VERSION=1.2.3+45 \
  firmware
make \
  GCC_PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin \
  KEY_FILE=/secure/path/production-ecdsa-p256.pem \
  verify
```

注意：

- `KEY_FILE` 同时决定 MCUboot 内置的公钥和应用镜像的签名。两者必须来自同一把密钥。
- Makefile 只会在默认开发密钥缺失时生成 `.keys/development-ecdsa-p256.pem`；自定义 `KEY_FILE` 不存在时构建会直接失败，不会静默创建新的生产信任根。生产流水线仍应校验密钥文件和预期公钥指纹。
- 当前工程位于 `/mnt/e` 时，挂载权限可能无法真正落实 `chmod 600`；生产私钥应放在 Linux 原生受控文件系统、HSM 或专用密钥服务中，不要依赖工作区内文件权限。
- 私钥不得放入版本库或随设备交付。只分发签名后的 `.bin` 或完整的 `factory.hex`。
- 已烧入旧公钥的 MCUboot 不会接受由另一把私钥签名的升级包。更换信任根时必须采用经过审核的 Bootloader 更新/工厂重刷流程。

## 3. MCUboot USB CDC + mcumgr 升级

### 3.1 应用串口自动切换到 Recovery

一键烧写不检查 BOOT0，不要求重新上下电，也不要求短按 RESET。上传器枚举当前可用串口，不先
按 VID/PID 或产品名排除端口，并按以下顺序进行协议识别：

1. 先用短超时 `image list` 判断端口是否已经运行 MCUboot SMP；若是，立即开始烧写。
2. 用户显式设置 `MCUMGR_PORT` 时，该端口就是本次操作授权。上传器先记录其物理 USB 绑定，再按
   PX4 `Tools/px4_uploader.py` 的 MAVLink 顺序执行三轮 bootloader kick：清空串口缓冲、发送 MAVLink v1
   broadcast `0/0` 和 targeted `1/0`。
3. 未显式指定端口时使用安全自动发现：通过固定版本 pymavlink 请求 `HEARTBEAT` 与
   `AUTOPILOT_VERSION`。当前应用必须同时匹配 system/component
   `1/1`、Ground Rover、PX4 autopilot、固件版本 `0.1.0`、板版本 `1`、能力位和非零硬件 UID；识别后
   发送三轮 broadcast/targeted MAVLink reboot。Windows 在一个 PowerShell `SerialPort` 实例内完成
   全部二进制写入，POSIX 在一个 raw file descriptor 内完成，避免每一帧重新开关端口。
4. 应用确认当前未 armed 后，在 RTC 备份寄存器写入一次性 Recovery 请求并执行
   `NVIC_SystemReset()`。
5. MCUboot 读取该请求后持续运行 USB Recovery，不受普通 3 秒窗口限制；USB 初始化成功后请求被
   清除，因此上传结束时的 `mcumgr reset` 会正常启动应用。

显式端口路径不先发送身份请求，因为明确端口本身就是操作授权；它仍只发送当前应用支持的 MAVLink
重启命令，复位必须经过 Commander。自动发现路径必须先取得完整的当前 MAVLink 复合身份。

软件重启后只接受与已选应用相同的物理 USB 身份进入 Recovery，禁止无约束扫描所有端口，以免并发
接入另一块板时误写。Windows 原生进程从注册表优先绑定 Container ID，并用 Configuration Manager
排除离线历史 devnode；同一 COM 仍有多个候选时拒绝猜测。注册表缺项按端口用当前
`Win32_SerialPort.PNPDeviceID` 的完整实例 ID 补齐，不能因其他 COM 已解析就跳过目标端口。WSL 使用
同一完整 PnP 实例 ID。POSIX 绑定 USB serial、回退到物理总线拓扑，因此允许同一设备的 COM/tty
名称变化。无法建立唯一物理绑定时命令会安全失败。

PX4 uploader 不等待应用 ACK，而以重枚举后的 Bootloader 握手为准；显式端口路径采用同一判据。
安全自动发现路径还会解析当前固件的定向 accepted 与 `result_param2=3`；明确的
DENIED/REJECTED ACK 立即终止。无论 ACK 是否到达，只有相同物理 USB 绑定重新
枚举且 `image list` 得到有效 MCUboot SMP 响应，才判定 Recovery 切换成功。软件入口无效时命令返回
非零，不把仅完成串口写入误报为复位成功。

普通上电且没有软件 Recovery 请求时仍保留原有 3 秒 SMP 窗口。Primary 镜像缺失、签名无效或
向量表无效时，MCUboot 也会持续停留在 Recovery。正在运行但不提供上述 MAVLink 身份与重启命令的
应用无法安全授权软件切换；首次迁移必须通过现有工厂/调试通道写入
`build/H743_FreeRTOS_factory.hex`。普通 `mcumgr` 只更新应用槽，不能更新 MCUboot 本身。从完整
Factory 镜像部署完成后，后续升级不再需要断电或按键操作。

PX4 式软件切换的前提是应用命令链仍能运行。若 USB CDC 仍枚举但应用已经不产生 heartbeat、也不处理
定向 reboot 命令，USB 主机无法仅凭串口字节强制 Cortex-M7 执行系统复位；上传器会失败关闭，不会
盲发后继续烧写。固件现已接入约 2048 ms 的独立 IWDG：appMain 是应用侧唯一 feed owner，只有
BootHealth 持续推进安全健康 generation 才喂狗；MCUboot 对跨复位仍运行的 IWDG 临时扩展，并在
Recovery、镜像校验、swap、Flash 与 USB 串行长循环中喂狗。应用在冷启动时先启动 IWDG/LSI，再写
PR/RLR/WINR 并等待同步；若先等待 SR 清零再启动，LSI 未运行会导致初始化永久失败。该链路的源码/目标构建证据不等于实际
2 秒复位、复位后 PWM/GPIO 或长时间升级行为已经验收；无按键故障恢复仍必须以实板 IWDG 试验或主机
可控的 NRST/电源控制为准，不能由 uploader 主机脚本单独保证。

MCUboot 完成 swap 和 Primary 校验后不会携带当前 USB、PLL、SysTick 或 NVIC 状态热跳转应用。
它写入一次性应用桥接标记并执行系统复位；冷上电、全片擦除或 ROM DFU 后 D3 尚未由 Application
初始化时，MCUboot 先以当前 `RCC->RSR` 建立最小 v2 bridge 记录，避免形成“Bootloader 等待
Application 初始化、Application 又无法启动”的循环依赖。复位后的 Bootloader 清除标记，只读复验
Primary 的 header、哈希、ECDSA 签名和 `0x08040400` 向量后再进入应用。该复验不处理 swap trailer，
因此 pending/test 镜像能获得首次启动机会；真实的桥接写入或复验失败仍停留在 Recovery，不会形成复位循环。

2026-08-19 实板 ROM DFU 首启验证发现：诊断 Flash 全 `0xFF` 且 Primary 已 confirmed 时若仍永久停在
Recovery，应先检查 D3 bridge 头的冷启动初始化，而不是把 Recovery 本身误判成 Application USB 死机。
随后捕获的真实记录为 `ERROR_HANDLER / APPLICATION_RUNNING / stacked_r0=2048`，对应应用 IWDG 在
LSI 启动前等待 SR 同步。两项修复烧写后，USB 完成 MCUboot→bridge reset→Application 双枚举，
`--preflight-only` 返回 `Dima Rover MAVLink`；这证明首启和协议链恢复，不替代 IWDG 超时及 PWM 板测。

Linux 一般为 `/dev/ttyACM0`，Windows 一般为 `COMx`。下面所有命令中的端口必须替换为实际端口：

```bash
mcumgr --conntype serial \
  --connstring "dev=/dev/ttyACM0,baud=115200" \
  image list
```

Windows PowerShell 示例：

```powershell
mcumgr --conntype serial `
  --connstring "dev=COM7,baud=115200" `
  image list
```

`baud=115200` 是 CDC line-coding 参数，实际链路仍是 USB FS。手工命令要求设备已经处于
Recovery；日常升级应直接使用下一节的一键命令，由上传器自动完成应用识别和软件切换。

### 3.2 一条命令完成构建与上传

工程的正式构建和上传入口必须运行在 Windows 原生 GNU Make、Windows 路径和 Windows Python
中；WSL 只可作为调用 Windows 进程的控制终端，不能直接执行本工程的 Make 构建。根 Makefile
优先选择 `%USERPROFILE%\.platformio\penv\Scripts\python.exe`。主机只需预先具备 Windows GNU Make
与 Python 3 + pip；如果没有显式提供 `GCC_PATH`、Go 或 `mcumgr`，脚本会自动准备固定版本的
xPack Arm GNU 10.3.1、Go 和带 Dima USB CDC 修补的 Apache `mcumgr`，并按锁文件准备仅用于协议
编解码的 pymavlink。

所有自举工具均进入 `Path.home()/.cache/dima-rover/host-tools` 共享缓存，不写入仓库、系统工具
目录或个人 Go `bin` 目录。下载必须同时通过固定版本、文件大小和 SHA-256 校验：Go 优先使用
国内镜像并保留官方回退，Arm GNU 优先使用国内 GitHub 代理并保留 GitHub 回退。后续构建直接
命中缓存。在工程根目录执行：

```bash
make dima_rover upload
```

日常 OTA 会进入快速调度：只执行一次真实 Make 依赖图，不再为了构建进度条额外执行“生成物稳定化 +
全量 dry-run 规划”。这不会跳过生成器、源码重编译、ELF 生命周期检查、签名或上传镜像验证；任何输入
过期仍会在打开设备前完成重建和校验。架构门禁使用源码与配置内容指纹缓存：相关文件新增、删除或内容
变化会自动失效并重新执行完整架构检查；单独执行 `make dima_rover`、`make verify`、`make firmware`、
`make app-check` 或 `make check-architecture` 时仍强制实时检查。完整 Factory 发布流程不走快速调度。

上传日志的每个 `[STAGE]` 现在同时打印 `t=`（从命令开始的累计耗时）和 `delta=`（距上一个阶段的
耗时），可直接区分主机准备、Recovery/应用重枚举和镜像传输。当前开发阶段的主机上传流程不再等待或
探测 `image_ok`；板端仍可在健康条件满足后自行确认，但该状态不再作为上传命令的成功门禁。

该命令在用户未显式传入 `-jN` 时自动采用 `-j4`，只构建上传必需的应用 ELF/BIN、签名镜像并缓存
应用 ELF 与签名校验结果；不会为日常 OTA 重建 MCUboot、Factory HEX 或重跑完整 Factory 布局验收。
`make dima_rover` 单独执行时仍保留完整发布验收。C 源码的 GCC 汇编 listing 默认关闭；需底层排查时
先执行 `make clean`，再设置 `DIMA_LISTINGS=1` 完整重建。上传器默认选择
`build/H743_FreeRTOS_signed.bin`，解析本地签名镜像 SHA-256，
扫描并识别应用或 Recovery 串口；主机工具只解析一次，设备探测不再在构建前额外重复一轮独立
USB 预检。板上已运行相同 active/confirmed hash 时默认跳过重写并恢复应用运行。
当前开发阶段不会因 Primary 仍是 active 但未 confirmed 的测试镜像而阻止上传；再次上传会直接覆盖
Secondary，因此旧镜像的回滚副本也会随之丢失。签名校验、必要的 TEST 请求、复位和应用身份闭环不变。

需要更新时，状态机依次执行 `UPLOAD_SECONDARY`、`TEST`、`RESET` 和 MAVLink 应用身份校验。
`image test <hash>` 仍是请求 MCUboot 在下次启动交换 Secondary 的必要动作；主机只要求该命令成功，
不再把某个版本 `image list` 文本中的 `pending` 标志作为上传成功门禁。应用成功重枚举并通过身份校验
后上传命令即成功，不再复位探测 Primary 的 confirmed 状态。仓库自举的是带 Dima USB CDC 快速通道的
固定版本 `mcumgr`：在虚拟 921600 波特率下
取消 Apache 串口传输原有的 20 ms 分片间延时、将 NLIP 帧扩展到 512 字节 MTU，并把非末尾固件
块对齐到 STM32H743 的 32 字节 Flash 写入粒度。生产默认使用 `--maxwinsize 1` 的 stop-and-wait；
它规避 Bootloader 2048 字节 CDC RX 环形缓冲的窗口溢出，并已取消原来把单窗口传输压低到约
2 KiB/s 的主机限速路径。实际整包速率以结束时打印的字节数、耗时和 KiB/s 为准。

Linux 会枚举 `/dev/serial/by-id/`、`/dev/ttyACM*` 和 `/dev/ttyUSB*`，并优先显示稳定的
`by-id` 路径；WSL 使用 Windows .NET 串口 API 枚举当前 `COMx`，通过 PowerShell 访问应用，
再使用自动缓存的 Windows `mcumgr.exe` 完成 Recovery 传输。VID/PID、COM 号和设备名称都不作为
烧写授权，只有当前 MAVLink 复合身份或 MCUboot `image list` 协议响应才确认
身份；MAVLink 的 64-bit hardware UID 用于约束复位前后的应用关联，但不作为密码学设备身份。普通
端口打开失败或协议不匹配只会被跳过；
若识别到多个 Dima 协议端点，命令会拒绝猜测目标，此时必须覆盖端口：

```bash
make dima_rover upload MCUMGR_PORT=/dev/ttyACM1
make dima_rover upload MCUMGR=/absolute/path/to/mcumgr
```

`UPLOAD_IMAGE` 可覆盖默认上传包；外部包与本地 ELF stamp 无关联，因此每次都会用 `KEY_FILE` 对
实际上传文件重新执行签名验证。`UPLOAD_WAIT_SECONDS` 可覆盖默认 60 秒等待时间。`UPLOAD_FORCE=0`
是默认值，相同 active/confirmed hash 不再重复擦写；确需强制重写时显式设置 `UPLOAD_FORCE=1`，该特殊
路径会额外读取一次 Secondary，避免 Primary 与 Secondary 同 hash 时产生歧义。主机不再提供或执行
confirmed 状态验收；如需确认 `image_ok`，应按 3.3 节单独检查。`MCUMGR_BAUD`、`MCUMGR_MTU`
和 `MCUMGR_MAX_WINDOW` 分别覆盖虚拟波特率、串行 MTU 和窗口；窗口默认值为 1，在完成持续实板压力
验收前不应调大。`HOST_TOOLS_CACHE_ROOT` 可覆盖包括 Arm GNU、Go、`mcumgr`、pymavlink 和 Python
依赖在内的统一工具缓存目录。受限网络可通过
`DIMA_GOPROXY` 指定 Go 模块代理；离线环境也可用 `GCC_PATH` 和
`MCUMGR=/absolute/path/to/mcumgr` 显式指定已审核工具。无论使用哪个入口，上传包都必须由板上
MCUboot 已内置公钥对应的私钥签名。

### 3.3 手工升级流程

以下命令以 Linux 端口为例。本工程统一显式使用 **`-n 2`**，将 direct-image 编号 `2`
映射到 Secondary slot。为兼容旧客户端，省略该字段时发送的默认编号 `0` 也会安全回退到
Secondary；Bootloader 会拒绝 `-n 1`，从实现层禁止 USB 擦写正在运行的 Primary slot。

1. 进入 recovery，并查看当前镜像：

   ```bash
   mcumgr --conntype serial \
     --connstring "dev=/dev/ttyACM0,baud=115200" \
     image list
   ```

2. 上传签名镜像到 Secondary slot：

   ```bash
   mcumgr --conntype serial \
     --connstring "dev=/dev/ttyACM0,baud=115200" \
     image upload -n 2 --maxwinsize 1 build/H743_FreeRTOS_signed.bin
   ```

3. 再次列出镜像，复制 Secondary 镜像显示的完整 SHA-256 hash：

   ```bash
   mcumgr --conntype serial \
     --connstring "dev=/dev/ttyACM0,baud=115200" \
     image list
   ```

4. 将该 hash 标记为下一次测试启动镜像：

   ```bash
   mcumgr --conntype serial \
     --connstring "dev=/dev/ttyACM0,baud=115200" \
     image test <secondary-image-hash>
   ```

5. 复位。排障时可以先执行一次 `image list` 观察 MCUboot 状态，但其 `pending` 文本不是主机成功条件：

   ```bash
   mcumgr --conntype serial \
     --connstring "dev=/dev/ttyACM0,baud=115200" \
     reset
   ```

6. 复位后保持供电稳定，等待 MCUboot 完成 scratch swap 并启动新应用。不要在交换 Flash 的过程中断电。

7. 新应用启动 FreeRTOS 和 USB 后，由 HP 工作队列上的 `BootHealthService` 开始连续 5 秒稳定窗口。只有 Parameter Core ready、Commander healthy，并且 `actuator_armed → vehicle_control_mode → vehicle_status` 三个安全 Topic 具有相同时间戳、状态一致、时间新鲜且相对上一组严格前进，窗口才继续累计；任一条件失效都从零重新计时。系统不创建示例心跳 Topic。窗口满足后仍须处于 DISARMED 且 Flash 非 busy，才写入 MCUboot `image_ok`；ARMED 或 FlashBusy 只返回 Deferred，并在条件解除后重试。它不是 `vTaskDelay(5000)` 后的无条件确认。

8. 如需验收确认状态，复位并在 3 秒窗口内再次执行 `image list`，确认运行镜像已经是新版本且为 confirmed/permanent 状态。

### 3.4 回滚行为

- 新镜像在 5 秒健康窗口内崩溃、看门狗复位或被人工硬复位，或者 Parameter/Commander/三个安全 Topic 未形成连续健康进展时，`image_ok` 尚未写入；下一次 MCUboot 启动会回滚到旧镜像。
- 如果新镜像只是无限卡死且没有看门狗触发，需人工复位/重新上电，之后 MCUboot 才能执行回滚。
- 回滚和首次测试启动都涉及 Flash swap，必须保持电源和 USB 供电稳定。
- `mcumgr` recovery 依赖 MCUboot 本身能够运行；MCUboot 损坏时改用下一节的 ROM DFU。

### 3.5 Fault 冷启动持久化

- Application HardFault/Panic 只更新 non-cacheable D3 固定故障快照和 capture-valid marker，执行内存屏障后立即复位；异常上下文禁止扫描、擦除或编程 Bank 1 Flash。
- MCUboot 在冷启动阶段校验 D3 snapshot，按既有 ABI、序号和 CRC 生成持久 Flash record，写入诊断扇区并进入 USB Recovery。诊断 Flash store 只属于 MCUboot，最终 Application ELF 不得链接其 enable/capture/store 符号。
- 正常 Parameter/BootControl Flash transaction、Armed/Flash coordinator 和 Fault emergency capture 是三条独立路径；Fault 记录不能等待 Mutex、WorkQueue、USB 或 Application Runtime shutdown。
- D3 record 跨 Application Runtime 保留，但 Runtime 重启不得把上一轮 Parameter/uORB/RC cache 当作有效状态。故障被 MCUboot 成功持久化和处理后，才按既有协议清除 pending 标志。

## 4. STM32CubeProgrammer ROM USB DFU 最终救援

### 4.1 进入 ROM Bootloader 的必要条件

STM32H743 使用 AN2606 的 Pattern 10。正常量产配置应满足：

- `BOOT_ADD0 = 0x0800`：BOOT0 低时从用户 Flash/MCUboot 启动。
- `BOOT_ADD1 = 0x1FF0`：BOOT0 高时进入 System Memory ROM Bootloader。
- `SWAP_BANK = disabled`：本工程按正常 Bank1/Bank2 地址映射执行 swap 和擦除，不能启用 Bank swap。
- 进入救援时将 BOOT0 拉高，然后硬复位或重新上电。
- 绝对不要设置不可逆的 RDP Level 2。RDP2 下芯片不会从 System Memory 启动，ROM DFU 最终救援通道将永久失效。
- 不要让生产烧写包修改 `BOOT_ADD1`。本工程的 `build/H743_FreeRTOS_factory.hex` 不包含 Option Bytes。

建议生产治具在出厂前用 STM32CubeProgrammer读取并记录 Option Bytes，而不是只依赖默认值。

### 4.2 硬件约束（STM32H743VI LQFP100）

| 信号 | MCU 引脚 | LQFP100 管脚 | 要求 |
| --- | --- | ---: | --- |
| BOOT0 | BOOT0 | 94 | 默认可靠下拉；必须留测试点、跳帽或受控电路，故障时可可靠拉高，不能悬空 |
| USB DM | PA11 / USB_OTG_FS_DM | 70 | 必须连接到用于救援的 USB 口 |
| USB DP | PA12 / USB_OTG_FS_DP | 71 | 必须连接到用于救援的 USB 口 |

ROM Bootloader 的 USB FS DFU 使用 HSI48 + CRS，不依赖外部 HSE；芯片内部已提供 USB DP 上拉，不需要外接 DP 上拉电阻。硬件还必须满足：

- USB 收发器电源域必须得到有效的约 3.3 V 供电（数据手册正常工作范围为 3.0–3.6 V）。
- H743 ROM DFU 不支持 USBREGEN 模式；仅用 1.8 V 给芯片/USB 域供电时不能使用 ROM DFU，除非硬件另行提供符合要求的 3.3 V USB 电源域。
- USB D+/D- 走线、ESD 器件、串联电阻和连接器不能在“应用死机”时被外部开关、隔离器或未上电器件断开。
- BOOT0 高电平和硬复位/电源循环必须在不运行任何固件的情况下也可由人工或生产治具完成。

### 4.3 使用 STM32CubeProgrammer 恢复完整系统

1. 断电或保持 NRST 为低，把 BOOT0 拉高。
2. 连接 USB FS 数据线，然后重新上电或释放 NRST。
3. 打开 STM32CubeProgrammer，选择连接类型 **USB**，刷新端口并连接 DFU 设备。
4. 读取设备信息和 Option Bytes，确认 `BOOT_ADD1 = 0x1FF0`、`SWAP_BANK = disabled` 且不是 RDP Level 2。
5. 读取并记录 ROM Bootloader 版本。AN2606 Rev 70 记载 V9.1 存在 Bank2 erase 提前返回问题，V9.2 已修复；量产验收必须用实际芯片验证 Bank2 擦除和读回全 `0xFF`。若实际为 V9.1，Bank2 erase 后必须按 ST 的最坏擦除时间等待并读回验证，不能立即发送下一条 Flash 命令。
6. 执行全片擦除，清除 Primary、Secondary、Scratch 中可能残留的 pending/test trailer。若因保护设置先回退到 RDP0，注意该操作本身会触发全片擦除。
7. 打开 `build/H743_FreeRTOS_factory.hex`。HEX 已包含目标地址，无需手工填写下载地址。
8. 启用写后校验（Verify programming），执行 Download。确认 MCUboot 区 `0x08000000` 和签名 Primary 区 `0x08040000` 均校验成功。
9. 断开 STM32CubeProgrammer，断电或拉低 NRST，把 BOOT0 恢复为低电平。
10. 重新上电或释放 NRST。MCUboot 应先出现 3 秒 CDC recovery 窗口，随后启动位于 `0x08040400` 的应用。

全片擦除很重要：只覆盖 Factory HEX 涉及的地址而保留 Secondary/Scratch，旧的 MCUboot trailer 可能在下一次启动触发非预期 swap。

## 5. 量产与故障演练检查表

- [ ] 正常复位时 BOOT0 为低，设备先出现约 3 秒 MCUboot CDC 窗口，随后启动应用。
- [ ] 3 秒内执行 `mcumgr ... image list` 后，设备持续停留在 recovery。
- [ ] `image upload -n 2 build/H743_FreeRTOS_signed.bin`、`image test <hash>`、`reset` 全流程通过。
- [ ] 新镜像的 Parameter、Commander 和三个安全 Topic 连续健康 5 秒以上后保持不回滚。
- [ ] 在 5 秒内人工复位后，旧镜像能够恢复。
- [ ] 擦除或破坏 Primary 镜像后，MCUboot 能永久停留在 USB recovery。
- [ ] BOOT0 拉高并硬复位后，STM32CubeProgrammer 能通过同一 USB 口识别 ROM DFU。
- [ ] 已记录实际 ROM Bootloader 版本，并验证 Bank2 全片擦除和读回。
- [ ] ROM DFU 全片擦除并烧写 `build/H743_FreeRTOS_factory.hex` 后，BOOT0 拉低可完整启动系统。
- [ ] `BOOT_ADD1` 保持 `0x1FF0`、`SWAP_BANK` 为 disabled，生产包不含 Option Bytes，且从未启用 RDP Level 2。
- [ ] 生产镜像使用受控的 `KEY_FILE`，私钥不存在于仓库和交付包中。

## 6. 依据

- ST AN2606 Rev 70，STM32H74xxx/75xxx System Memory Bootloader、Pattern 10、USB DFU 引脚/供电和 ROM 版本限制。
- ST DS12110 Rev 11，STM32H743VI LQFP100 pinout 与 USB OTG FS 电气要求。
- 本工程 `Bootloader/Src/main.c`、`Bootloader/Src/flash_map_backend.c`、`Dima/platform/stm32h7/system/BootControl.cpp`、`Dima/platform/stm32h7/flash/flash_bank1.c`、`Dima/modules/boot_health/BootHealthService.cpp`、Commander 三个安全消息、`Boards/H743/Inc/boot_layout.h` 和根 Makefile。
