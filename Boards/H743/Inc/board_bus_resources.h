#pragma once

#include "main.h"

/* H743 板级资源别名：底层 SPI/EXTI 后端只认识“总线资源”，不直接依赖
 * ICM42688P 这类具体设备名。这里仅完成引脚角色映射；外设实例的唯一所有权
 * 仍由 platform_composition 与 H743 板级合同决定，禁止驱动绕过该层重复初始化。 */
#define DIMA_SPI4_DEVICE_CS_GPIO_Port ICM42688_CS_GPIO_Port
#define DIMA_SPI4_DEVICE_CS_Pin ICM42688_CS_Pin
#define DIMA_INTERRUPT_SOURCE1_Pin ICM42688_INT1_Pin
#define DIMA_INTERRUPT_SOURCE2_Pin ICM42688_INT2_Pin
