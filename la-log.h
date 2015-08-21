#ifndef _LA_LOG_H
#define _LA_LOG_H

#include <syslog.h>
#include <stdarg.h>
#include <stdio.h>

void la_log_syslog_open();
void la_log(int priority, const char *format, ...);

#endif
