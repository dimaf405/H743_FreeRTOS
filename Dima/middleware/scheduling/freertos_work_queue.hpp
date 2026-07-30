#pragma once

#include "Dima/middleware/scheduling/work_queue.hpp"

namespace dima::middleware::scheduling {

// Task context only. Call before scheduling work on either default queue.
// Idempotent: each static worker is created at most once. False means another
// call is initializing, or a partial creation failed; retry later without
// busy-spinning at a priority that can starve the initializer.
bool init_default_work_queues();
WorkQueue &hp_default_work_queue();
WorkQueue &lp_default_work_queue();

#if defined(APP_FREERTOS_WORK_QUEUE_TEST_SEAM)
// Drives one real backend scan/wait iteration without entering the worker's
// infinite task loop. Test-only; absent from production declarations.
bool freertos_work_queue_test_run_one_step(QueueClass queue);
// Requires a quiescent test state with no managed work item or active worker.
void freertos_work_queue_test_reset();
#endif

} // namespace dima::middleware::scheduling
