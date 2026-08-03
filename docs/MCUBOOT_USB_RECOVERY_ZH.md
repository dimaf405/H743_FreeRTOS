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

### 3.1 连接和 3 秒恢复窗口

BOOT0 保持低电平，连接 USB FS 口并复位设备。MCUboot 枚举为 CDC 串口后等待有效 SMP 请求 3 秒：

- 3 秒内收到任意有效 `mcumgr` 请求后，设备会持续停留在 MCUboot recovery，直到执行 `mcumgr reset`。
- 3 秒内没有收到请求且 Primary 镜像有效时，MCUboot 启动应用。
- Primary 镜像缺失、签名无效或向量表无效时，MCUboot 会永久停留在 recovery，此时没有 3 秒限制。

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

`baud=115200` 是 CDC line-coding 参数，实际链路仍是 USB FS。若端口只在复位后短暂出现，先准备好命令，复位后立即执行 `image list`；首次有效响应会锁定 recovery 会话。

### 3.2 一条命令完成构建与上传

一次性安装固定版本的 Apache `mcumgr`，并确保生成目录位于 PATH：

```bash
go install github.com/apache/mynewt-mcumgr-cli/mcumgr@v0.0.0-20221004073047-5c56bd24066c
```

Linux 默认生成到 `$(go env GOPATH)/bin/mcumgr`；Windows 默认生成到
`%USERPROFILE%\go\bin\mcumgr.exe`，本工程在 WSL 中也会自动寻找后者。编译环境还需要 GNU
Make、Python 3 + pip，以及 PATH 中的 `arm-none-eabi-gcc` 工具链；也可继续通过 `GCC_PATH`
显式指定工具链目录。依赖准备好后，在工程根目录只需执行：

```bash
make dima_rover upload
```

该命令会依次完成应用与 MCUboot 编译、签名和布局校验，默认选择当前构建目录中的
`build/H743_FreeRTOS_signed.bin`，等待 `H743 MCUboot Recovery` USB CDC 端口出现，随后自动执行
Secondary 上传、读取本地签名镜像 SHA-256、设置测试启动和复位。命令提示等待设备时保持 BOOT0
为低并按下 RESET；首次有效请求会使 Bootloader 保持在 recovery。

Linux 只会自动选择 USB 标识为 VID `0483`、PID `5740`、产品名
`H743 MCUboot Recovery` 的 CDC 端口，并优先显示稳定的 `/dev/serial/by-id/` 路径。在 WSL
中，如果 Windows PATH 或 `%USERPROFILE%\go\bin` 中存在 `mcumgr.exe`，命令会查找相同
VID/PID 的 `COMx` 并通过短超时 `image list` 确认 Recovery；VID/PID 本身不能区分应用 CDC
和 Bootloader。检测到多个候选设备时命令会拒绝猜测目标，此时必须覆盖端口：

```bash
make dima_rover upload MCUMGR_PORT=/dev/ttyACM1
make dima_rover upload MCUMGR=/absolute/path/to/mcumgr
```

`UPLOAD_IMAGE` 可覆盖默认上传包，`UPLOAD_WAIT_SECONDS` 可覆盖默认 60 秒等待时间。无论使用
哪个入口，上传包都必须由板上 MCUboot 已内置公钥对应的私钥签名。

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
     image upload -n 2 build/H743_FreeRTOS_signed.bin
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

5. 用 `image list` 确认新镜像已处于 pending/test 状态，然后复位：

   ```bash
   mcumgr --conntype serial \
     --connstring "dev=/dev/ttyACM0,baud=115200" \
     image list

   mcumgr --conntype serial \
     --connstring "dev=/dev/ttyACM0,baud=115200" \
     reset
   ```

6. 复位后保持供电稳定，等待 MCUboot 完成 scratch swap 并启动新应用。不要在交换 Flash 的过程中断电。

7. 新应用启动 FreeRTOS 和 USB 后，由 HP 工作队列上的 `BootHealthService` 开始 5 秒稳定窗口；LP 工作队列上的 `HelloWorld` 默认每 1000 ms 发布一次 `app_heartbeat`。只有稳定窗口已满 5 秒且健康服务至少复制到一个新 heartbeat，才会写入 MCUboot `image_ok` 确认标记。它不是 `vTaskDelay(5000)` 后的无条件确认；USB 输出失败也不会阻断 heartbeat。满足这两个门槛后，下一次复位不会回滚，无需另行执行 `mcumgr image confirm`。

8. 如需验收确认状态，复位并在 3 秒窗口内再次执行 `image list`，确认运行镜像已经是新版本且为 confirmed/permanent 状态。

### 3.4 回滚行为

- 新镜像在 5 秒健康窗口内崩溃、看门狗复位或被人工硬复位，或者稳定窗口结束前始终没有产生 `app_heartbeat` 时，`image_ok` 尚未写入；下一次 MCUboot 启动会回滚到旧镜像。
- 如果新镜像只是无限卡死且没有看门狗触发，需人工复位/重新上电，之后 MCUboot 才能执行回滚。
- 回滚和首次测试启动都涉及 Flash swap，必须保持电源和 USB 供电稳定。
- `mcumgr` recovery 依赖 MCUboot 本身能够运行；MCUboot 损坏时改用下一节的 ROM DFU。

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
- [ ] 新镜像运行 5 秒以上且 `app_heartbeat` 正常发布后保持不回滚。
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
- 本工程 `Bootloader/Src/main.c`、`Bootloader/Src/flash_map_backend.c`、`Dima/adapters/mcuboot/mcuboot_app.c`、`Dima/modules/boot_health/boot_health.cpp`、`Dima/messages/app_heartbeat.hpp`、`Boards/H743/Inc/boot_layout.h` 和根 Makefile。
