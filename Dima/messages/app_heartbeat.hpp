#pragma once

#include "uorb/uORB.hpp"

#include <stdint.h>

struct app_heartbeat_s {
    uint64_t timestamp_us;
    uint32_t sequence;
};

ORB_DECLARE(app_heartbeat);
