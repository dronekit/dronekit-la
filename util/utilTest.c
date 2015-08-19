#include <stdint.h>
#include <stdio.h>
#include "util.h"


#include <time.h>
#include <sys/time.h>

#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

int main(int argc, char *argv[])
{

    char buf[32];
    uint64_t t;

    t = clock_gettime_us(CLOCK_REALTIME);

    printf("%s\n", clock_tostr_r(t, buf));

    return 0;

} // main
