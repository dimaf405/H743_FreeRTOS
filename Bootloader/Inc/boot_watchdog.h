#ifndef H743_BOOT_WATCHDOG_H
#define H743_BOOT_WATCHDOG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Extend an already-running application IWDG for MCUboot's bounded flash and
 * recovery work. This configures but does not start an inactive watchdog. */
void boot_watchdog_prepare(void);

/* Reload an IWDG that may still be active after an application watchdog
 * reset. Writing the reload key is harmless when the watchdog is inactive. */
void boot_watchdog_feed(void);

#ifdef __cplusplus
}
#endif

#endif /* H743_BOOT_WATCHDOG_H */
