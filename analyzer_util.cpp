#include "analyzer_util.h"
#include <sys/time.h>

void format_timestamp(char *buf, const uint8_t buflen, const uint64_t T)
{
    struct tm *tmp;
    time_t t = T / 1000000;
    tmp = localtime(&t);
    ::strftime(buf, buflen, "%Y%m%d%H%M%S", tmp);
}

uint64_t now() {
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec*(uint64_t)1000000 + tv.tv_usec;
}
