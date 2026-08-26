#include "api/Services.hpp"

namespace dima::platform {
namespace {

Services *g_services{nullptr};

} // namespace

bool install_services(Services &value) noexcept
{
    /* 只允许第一次安装或对同一对象的幂等安装；拒绝运行中替换引用集合，避免
     * 已持有后端引用的模块与全局服务表指向不同对象。 */
    // Services 在调度器启动前一次性安装，随后跨 Application Runtime 只读共享。
    if (g_services != nullptr) {
        return g_services == &value;
    }
    g_services = &value;
    return true;
}

bool services_installed() noexcept { return g_services != nullptr; }
Services *try_services() noexcept { return g_services; }

Services &services() noexcept
{
    /* 这是启动合同断言而非可恢复分支：返回虚构服务会把根因推迟为随机空引用，
     * 因此未安装时保持 fail-stop，故障阶段由外层启动诊断定位。 */
    if (g_services == nullptr) {
        for (;;) {
        }
    }
    return *g_services;
}

} // namespace dima::platform
