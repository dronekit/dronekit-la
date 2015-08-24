#include "la-log.h"

bool la_log_syslog = false;

void la_log_syslog_open()
{
    openlog("dl", LOG_NDELAY, LOG_LOCAL1);

    la_log_syslog = true;
}

void la_log(int priority, const char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    if (la_log_syslog) {
        vsyslog(priority, format, ap);
    } else {
        vfprintf(stderr, format, ap);
        fprintf(stderr, "%s", "\n");
    }
    va_end(ap);
}
