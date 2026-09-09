#include "os/endian.h"


// 普通运行期实现从对应头文件移出；保持原状态、错误分支和计算顺序。

uint16_t h743_bswap16(uint16_t value)
{
    return (uint16_t)((value << 8) | (value >> 8));
}
