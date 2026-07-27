# H743 FreeRTOS 软件架构与边界

## 1. 项目代码结构

项目自有应用代码按职责组织，不采用外部飞控项目的目录或 ABI：

```text
App/
├── application/             启动入口、静态对象装配和启动顺序
├── domain/motor/            纯速度到 PWM 数值转换
├── features/
│   ├── hello_world/         1 Hz 输出和 heartbeat 发布
│   └── boot_health/         启动健康判定
├── messages/                跨功能共享的 POD 消息
├── runtime/
│   ├── lifecycle/           ModuleBase 与 ModuleManager
│   ├── scheduling/          静态工作队列和 ScheduledWorkItem
│   ├── messaging/           Topic、Publication、Subscription
│   ├── time/                平台时间
│   └── libc/                无堆 C/C++ 运行时支撑
└── adapters/
    ├── usb_console/         USB CDC、stdio 和 _write 接线
    └── mcuboot/             运行镜像确认和 Flash 适配

Boards/H743/                 板级初始化和共享 Flash 布局
Bootloader/                  独立 MCUboot 镜像
Core/、USB_DEVICE/           CubeMX 生成层
Drivers/、Middlewares/       厂商及第三方代码
Linker/、make/、tools/       链接、构建和镜像工具
```

## 2. 依赖规则

- `App/application` 是唯一装配根，可依赖 features、runtime、adapters 和生成层入口。
- `App/features` 只依赖 messages、runtime 和明确的 C 适配器接口，不直接访问 HAL 外设句柄。
- `App/domain/motor` 是纯 C++17 算法，不依赖 HAL、FreeRTOS、USB、MCUboot、messages、runtime 或 application。
- `App/messages` 只定义共享 POD 数据。
- `App/runtime` 不依赖 feature、adapter 或 application；其中 FreeRTOS 依赖只存在于运行时实现。
- `App/adapters` 连接项目代码与 HAL、USB Device、FreeRTOS、MCUboot，不反向装配 feature。
- `Boards/H743` 只负责板级向量表、外设初始化顺序和共享 Flash 布局，不依赖 App。
- 跨组件 include 使用从项目根开始的 `App/...` 或 `Boards/...` 路径；组件内部头文件可以使用本目录相对路径。
- `App/` 和 `Boards/` 禁止包含 `main.h`，也禁止直接读取或写入 `hfdcan*`、`huart*` 等 HAL 全局外设句柄。

生成层只保留两处项目接线：`Core/Src/main.c` 调用 Boards 与 application 的 C 入口；USB Device 的 `USER CODE` 区调用 USB Console adapter。禁止在 CubeMX 生成区写入业务实现。

## 3. 启动与运行链

启动顺序固定为：

1. `board_vector_table_init()` 设置 `SCB->VTOR = 0x08040400` 并执行 DSB/ISB；
2. `HAL_Init()`；
3. 系统时钟和外设公共时钟配置；
4. `board_init()`；
5. `osKernelInitialize()` → `app_bootstrap_create()` → `osKernelStart()`；
6. `app_main_task()` 初始化 USB、HP/LP 工作队列、heartbeat Topic、`BootHealthService` 和 `HelloWorld`。

`board_init()` 的外设顺序固定为 GPIO → DMA → FDCAN1 → I2C2 → SDMMC1（可选）→ SPI4 → UART/USART → TIM。`BOARD_SD_INIT_AT_BOOT` 默认为 `0`。

## 4. 既有行为边界

- `HelloWorld` 默认每 1000 ms 从 LP 工作队列输出 `Hello World\r\n`，随后发布 `app_heartbeat_s`；USB 输出失败不影响 heartbeat。
- `BootHealthService` 在 HP 工作队列观察 5 秒稳定窗口，至少收到一个新 heartbeat 后才确认 MCUboot 测试镜像；USB 状态不是确认条件。
- ISR、HP 工作队列和故障入口禁止调用 `printf`。USB CDC 输出是 best-effort，未连接、忙或断开不得阻断系统运行。
- Module、ScheduledWorkItem、Topic、Publication 和 Subscription 保持静态生命周期；禁止运行期堆、动态任务、C++ exceptions 和 RTTI。
- `ScheduleClear()` 只撤销或更新调度状态，不是对象销毁前的静止屏障。
- 应用使用 full newlib；`_sbrk()` 必须 fail-closed，运行期仍不提供堆。

`App/domain/motor/speed_to_pwm.*` 仅完成固定六路归一化速度到 PWM 数值帧的转换，当前没有生产调用者。本阶段不包含 Mixer、ESC、arming、failsafe、PID 或 TIM 硬件输出。

## 5. 生成、构建与恢复边界

- 根目录 `H743_FreeRTOS.ioc` 是唯一 CubeMX 配置源；`240/240.ioc` 是历史副本。
- `Boards/`、`App/`、`Bootloader/`、`Linker/`、`GNUmakefile`、`make/project.mk`、`tools/`、`tests/` 和 `docs/` 为项目所有。
- 默认 `make` 读取 `GNUmakefile`，再由 `make/project.mk` 叠加项目源、C++、MCUboot 签名与验证规则；禁止用 `make -f Makefile` 绕过叠加层。
- `Boards/H743/Inc/boot_layout.h` 是应用和 Bootloader 共用的唯一 Flash 布局定义。
- MCUboot CDC + `mcumgr` 升级链和 ROM USB DFU + STM32CubeProgrammer 救援链不得因应用架构变化而改变。

权威校验命令：

```bash
make clean
make GCC_PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin verify
```

`clean` 与构建必须串行。操作和恢复要求见 [MCUboot USB 升级与恢复手册](MCUBOOT_USB_RECOVERY_ZH.md)。
