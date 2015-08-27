#ifndef _ANALYZER_UTIL
#define _ANALYZER_UTIL

#include <string>

// from: http://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf
#include <memory>
template<typename ... Args>
std::string string_format( const char* format, Args ... args )
{
    size_t size = snprintf( nullptr, 0, format, args ... ) + 1; // Extra space for '\0'
    std::unique_ptr<char[]> buf( new char[ size ] ); 
    snprintf( buf.get(), size, format, args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

#define streq(x,y) (!strcmp(x,y))

// inline float deg_to_rad(const float deg) {
//     return deg/M_PI * 180;
// }

// inline float rad_to_deg(const float rad) {
//     return rad*180/M_PI;
// }

#define deg_to_rad(x) (x/180*M_PI)
#define rad_to_deg(x) (x/M_PI * 180)

#define is_zero(x)  (x < 0.00001)

void format_timestamp(char *buf, uint8_t buflen, uint64_t T);

#endif
