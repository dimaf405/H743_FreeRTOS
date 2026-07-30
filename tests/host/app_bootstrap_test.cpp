#include "Dima/application/app_main.h"

#include "FreeRTOS.h"
#include "task.h"

#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {

struct CreateRecord {
    TaskFunction_t entry;
    const char *name;
    uint32_t stack_depth;
    void *argument;
    UBaseType_t priority;
    StackType_t *stack;
    StaticTask_t *task_buffer;
};

CreateRecord g_records[2]{};
unsigned g_create_calls{0U};
unsigned g_task_count_calls{0U};
unsigned g_task_lookup_calls{0U};
unsigned g_failures{0U};
UBaseType_t g_task_count{0U};
bool g_lookup_existing_task{false};
bool g_fail_first_create{false};
int g_existing_task_storage{0};

void expect(bool condition, const char *message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++g_failures;
    }
}

} // namespace

extern "C" void app_main_task(void *argument)
{
    (void)argument;
}

extern "C" TaskHandle_t xTaskCreateStatic(TaskFunction_t entry,
                                            const char *name,
                                            uint32_t stack_depth,
                                            void *argument,
                                            UBaseType_t priority,
                                            StackType_t *stack,
                                            StaticTask_t *task_buffer)
{
    if (g_create_calls < 2U) {
        g_records[g_create_calls] = {
            entry, name, stack_depth, argument, priority, stack, task_buffer,
        };
    }
    ++g_create_calls;

    if (g_fail_first_create && g_create_calls == 1U) {
        return nullptr;
    }
    return static_cast<TaskHandle_t>(task_buffer);
}

extern "C" UBaseType_t uxTaskGetNumberOfTasks(void)
{
    ++g_task_count_calls;
    return g_task_count;
}

extern "C" TaskHandle_t xTaskGetHandle(const char *task_name)
{
    ++g_task_lookup_calls;
    expect(task_name != nullptr && std::strcmp(task_name, "appMainTask") == 0,
           "bootstrap must query the CubeMX task by its generated name");
    return g_lookup_existing_task
        ? static_cast<TaskHandle_t>(&g_existing_task_storage)
        : nullptr;
}

void verify_create_record(const CreateRecord &record)
{
    expect(record.entry == &app_main_task,
           "the static task entry must be app_main_task");
    expect(record.name != nullptr &&
               std::strcmp(record.name, "appMainTask") == 0,
           "the static task name must match CubeMX appMainTask");
    expect(record.stack_depth * sizeof(StackType_t) == 2048U,
           "the static task stack must be exactly 2048 bytes");
    expect(record.argument == nullptr,
           "the app_main task must receive a null argument");
    expect(record.priority == 24U,
           "the bootstrap must preserve the legacy normal priority (24)");
    expect(record.stack != nullptr && record.task_buffer != nullptr,
           "the bootstrap must supply caller-owned stack and TCB storage");
}

void run_no_tasks_case()
{
    g_task_count = 0U;

    expect(app_bootstrap_create(),
           "bootstrap must create the task when no task lists exist yet");
    expect(g_task_count_calls == 1U,
           "bootstrap must inspect the task count once");
    expect(g_task_lookup_calls == 0U,
           "bootstrap must not query uninitialized task lists when count is zero");
    expect(g_create_calls == 1U,
           "bootstrap must make one static creation attempt");
    verify_create_record(g_records[0]);

    expect(app_bootstrap_create(), "successful bootstrap must be idempotent");
    expect(g_task_count_calls == 1U && g_create_calls == 1U,
           "cached bootstrap must not inspect or create tasks again");
}

void run_other_task_case()
{
    g_task_count = 1U;

    expect(app_bootstrap_create(),
           "bootstrap must create appMainTask when only another task exists");
    expect(g_task_count_calls == 1U && g_task_lookup_calls == 1U,
           "bootstrap must search initialized lists for appMainTask");
    expect(g_create_calls == 1U,
           "missing appMainTask must trigger one static creation");
    verify_create_record(g_records[0]);
}

void run_existing_task_case()
{
    g_task_count = 1U;
    g_lookup_existing_task = true;

    expect(app_bootstrap_create(),
           "bootstrap must adopt CubeMX's existing appMainTask");
    expect(g_task_count_calls == 1U && g_task_lookup_calls == 1U,
           "bootstrap must perform one guarded lookup");
    expect(g_create_calls == 0U,
           "adopting CubeMX appMainTask must not create a duplicate task");

    expect(app_bootstrap_create(), "adopted bootstrap must stay idempotent");
    expect(g_task_count_calls == 1U && g_task_lookup_calls == 1U &&
               g_create_calls == 0U,
           "adopted handle must be cached without another lookup or create");
}

void run_failure_retry_case()
{
    g_task_count = 0U;
    g_fail_first_create = true;

    expect(!app_bootstrap_create(),
           "a failed static task creation must be reported");
    expect(g_task_count_calls == 1U && g_task_lookup_calls == 0U &&
               g_create_calls == 1U,
           "failed no-task bootstrap must avoid lookup and attempt one create");

    expect(app_bootstrap_create(),
           "bootstrap must retry after a failed creation attempt");
    expect(g_task_count_calls == 2U && g_task_lookup_calls == 0U &&
               g_create_calls == 2U,
           "retry must recheck count and make one additional creation attempt");

    const CreateRecord &first = g_records[0];
    const CreateRecord &retry = g_records[1];
    verify_create_record(retry);
    expect(first.stack == retry.stack,
           "a retry must reuse the one dedicated static stack");
    expect(first.task_buffer == retry.task_buffer,
           "a retry must reuse the one static task control block");

    expect(app_bootstrap_create(),
           "bootstrap must stay successful after creation");
    expect(g_create_calls == 2U,
           "successful bootstrap calls must be idempotent");
    expect(g_task_count_calls == 2U,
           "cached retry success must not inspect task lists again");
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        std::fprintf(stderr,
                     "usage: app_bootstrap_test "
                     "<no-tasks|other-task|existing-task|failure-retry>\n");
        return 2;
    }

    if (std::strcmp(argv[1], "no-tasks") == 0) {
        run_no_tasks_case();
    } else if (std::strcmp(argv[1], "other-task") == 0) {
        run_other_task_case();
    } else if (std::strcmp(argv[1], "existing-task") == 0) {
        run_existing_task_case();
    } else if (std::strcmp(argv[1], "failure-retry") == 0) {
        run_failure_retry_case();
    } else {
        std::fprintf(stderr, "unknown bootstrap scenario: %s\n", argv[1]);
        return 2;
    }

    if (g_failures != 0U) {
        std::fprintf(stderr, "static bootstrap host test: %u failure(s)\n",
                     g_failures);
        return 1;
    }

    std::printf("static bootstrap host test passed: %s\n", argv[1]);
    return 0;
}
