#ifndef _ANALYZER_UTIL
#define _ANALYZER_UTIL

#include <string>
#include <string.h>

// from: http://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf
#include <memory>

template<typename ... Args>
std::string
string_format( const char* format, const Args ... args )
{
    
    uint32_t size = snprintf( nullptr, 0, format, args ... );
    if (size < 0) {
        ::fprintf(stderr, "snprintf error (%d): %s\n", size, strerror(errno));
        abort();
    }
    size += 1; // Extra space for '\0'
    std::unique_ptr<char[]> buf( new char[ size ] ); 
    snprintf( buf.get(), size, format, args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

template<typename ... Args>
std::string
string_format( const std::string format, const Args ... args )
{
    return string_format(format.c_str(), args ... );
}


#ifndef streq
#define streq(x,y) (!strcmp(x,y))
#endif

// inline float deg_to_rad(const float deg) {
//     return deg/M_PI * 180;
// }

// inline float rad_to_deg(const float rad) {
//     return rad*180/M_PI;
// }

#define deg_to_rad(x) ((x)*M_PI/180.0f)
#define rad_to_deg(x) ((x)*180.0f/M_PI)

#define is_zero(x)  (x < 0.00001)
#define is_equal(x, y)  (is_zero(fabs((x)-(y))))

void format_timestamp(char *buf, uint8_t buflen, uint64_t T);

uint64_t now();

double vec_len(double vec[3]);

#endif
