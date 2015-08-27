#include "analyzer_util.h"

void format_timestamp(char *buf, const uint8_t buflen, const uint64_t T)
{
    struct tm *tmp;
    time_t t = T / 1000000;
    tmp = localtime(&t);
    ::strftime(buf, buflen, "%Y%m%d%H%M%S", tmp);
}
