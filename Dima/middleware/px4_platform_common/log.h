#pragma once

#include <cstdio>

#ifndef PX4_INFO_RAW
#define PX4_INFO_RAW(...) std::printf(__VA_ARGS__)
#endif
