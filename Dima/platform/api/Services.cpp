#include "Services.hpp"

namespace dima::platform {
namespace {

Services *g_services{nullptr};

} // namespace

bool install_services(Services &value) noexcept
{
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
    if (g_services == nullptr) {
        for (;;) {
        }
    }
    return *g_services;
}

} // namespace dima::platform
