#if defined(__GNUC__)
__attribute__((noreturn))
#endif
void __cxa_pure_virtual(void)
{
#if defined(__arm__) || defined(__thumb__)
    __asm volatile("cpsid i" ::: "memory");
#endif
    for (;;) {
    }
}
