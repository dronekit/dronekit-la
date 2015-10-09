#ifndef UTIL_H
#define UTIL_H

#include <time.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __APPLE__

#define CLOCK_MONOTONIC 1

#endif

extern uint64_t clock_gettime_us(clock_t clock_id);
extern void clock_settime_us(clock_t clock_id, uint64_t t_us);
extern const char *clock_tostr_r(uint64_t t_us, char *buf);
extern const char *clock_gettime_str_r(clock_t clock_id, char *buf);
extern int hex_aton(char a);
#define MAC_STRING_LEN 18
extern char* mac_ntoa(uint8_t* mac_bytes, char* mac_string);
extern uint8_t* mac_aton(const char* mac_string, uint8_t* mac_bytes);

#ifdef __cplusplus
}
#endif

#endif /* UTIL_H */
