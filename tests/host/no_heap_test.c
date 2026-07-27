#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

void *_sbrk(ptrdiff_t increment);

static int check_rejected(ptrdiff_t increment)
{
    errno = 0;
    return _sbrk(increment) == (void *)(intptr_t)-1 && errno == ENOMEM;
}

int main(void)
{
    if (!check_rejected(0) || !check_rejected(1) || !check_rejected(-1)
        || !check_rejected(PTRDIFF_MAX)) {
        return 1;
    }
    puts("no-heap fail-closed host test passed");
    return 0;
}
