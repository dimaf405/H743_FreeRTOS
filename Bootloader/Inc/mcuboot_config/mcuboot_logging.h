#ifndef H743_MCUBOOT_LOGGING_H
#define H743_MCUBOOT_LOGGING_H

void boot_log_printf(const char *level, const char *fmt, ...);

#define MCUBOOT_LOG_ERR(...) boot_log_printf("E", __VA_ARGS__)
#define MCUBOOT_LOG_WRN(...) boot_log_printf("W", __VA_ARGS__)
#define MCUBOOT_LOG_INF(...) boot_log_printf("I", __VA_ARGS__)
#define MCUBOOT_LOG_DBG(...) do { } while (0)
#define MCUBOOT_LOG_SIM(...) do { } while (0)
#define MCUBOOT_LOG_MODULE_DECLARE(module)
#define MCUBOOT_LOG_MODULE_REGISTER(module)

#endif /* H743_MCUBOOT_LOGGING_H */
