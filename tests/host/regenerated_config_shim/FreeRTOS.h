#pragma once

#include "FreeRTOSConfig.h"

#include <stdint.h>

typedef int BaseType_t;
typedef unsigned int UBaseType_t;
typedef uint32_t StackType_t;

typedef struct StaticTask {
    uintptr_t opaque[8];
} StaticTask_t;
