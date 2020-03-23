#ifndef AP_MSGHANDLER_H
#define AP_MSGHANDLER_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h> // for abort()
#include <analyzer_util.h> // for is_zero

#include <math.h>

#define radians(x) (x/180*M_PI)
#include "Vector3f.h"
#include "Location.h"
#include "DataFlash/LogMessage.h"

#define LOGREADER_MAX_FIELDS 30

#ifndef streq
#define streq(x, y) (!strcmp(x, y))
#endif

class MsgHandler {
public:
    // constructor - create a parser for a MavLink message format
    MsgHandler(const struct log_Format &f);

    // retrieve a comma-separated list of all labels
    void string_for_labels(char *buffer, uint8_t bufferlen);

    bool field_value(const uint8_t *msg, const char *label, Vector3f &ret);

    // field_value - retrieve the value of a field from the supplied message
    // these return false if the field was not found
    template<typename R>
    bool field_value(const uint8_t *msg, const char *label, R &ret);

    bool field_value(const uint8_t *msg, const char *label,
		     char *buffer, uint8_t bufferlen);
    
    template <typename R>
    void require_field(const uint8_t *msg, const char *label, R &ret)
        {   
            if (! field_value(msg, label, ret)) {
                field_not_found(msg, label);
            }
        }
    void require_field(const uint8_t *msg, const char *label, char *buffer, uint8_t bufferlen);
    float require_field_float(const uint8_t *msg, const char *label);
    uint8_t require_field_uint8_t(const uint8_t *msg, const char *label);
    int32_t require_field_int32_t(const uint8_t *msg, const char *label);
    uint32_t require_field_uint32_t(const uint8_t *msg, const char *label);
    uint16_t require_field_uint16_t(const uint8_t *msg, const char *label);
    int16_t require_field_int16_t(const uint8_t *msg, const char *label);

private:

    void add_field(const char *_label, uint8_t _type, uint8_t _offset,
                   uint8_t length);

    void field_value_for_type_at_offset(const uint8_t *msg, uint8_t type,
                                        uint8_t offset, bool &ret);
    template<typename R>
    void field_value_for_type_at_offset(const uint8_t *msg, uint8_t type,
                                        uint8_t offset, R &ret);

    struct format_field_info { // parsed field information
        char *label;
        uint8_t type;
        uint8_t offset;
        uint8_t length;
    };
    struct format_field_info field_info[LOGREADER_MAX_FIELDS] = { };

    uint8_t next_field;
    size_t size_for_type_table[52]; // maps field type (e.g. 'f') to e.g 4 bytes

    void parse_format_fields();
    void init_field_types();
    void add_field_type(char type, size_t size);
    uint8_t size_for_type(char type);

protected:
    struct format_field_info *find_field_info(const char *label);

    template<typename R>
    void _field_value_for_type_at_offset(const uint8_t *msg, uint8_t type,
                                         uint8_t offset, R &ret);

    struct log_Format f; // the format we are a parser for
    ~MsgHandler();

    void location_from_msg(uint8_t *msg, Location &loc, const char *label_lat,
			   const char *label_long, const char *label_alt);

    void ground_vel_from_msg(uint8_t *msg,
			     Vector3f &vel,
			     const char *label_speed,
			     const char *label_course,
			     const char *label_vz);

    void attitude_from_msg(uint8_t *msg,
			   Vector3f &att,
			   const char *label_roll,
			   const char *label_pitch,
			   const char *label_yaw);
    void field_not_found(const uint8_t *msg, const char *label);
};

template<typename R>
bool MsgHandler::field_value(const uint8_t *msg, const char *label, R &ret)
{
    struct format_field_info *info = find_field_info(label);
    if (info == NULL) {
        return false;
    }

    uint8_t offset = info->offset;
    if (offset == 0) {
        return false;
    }

    field_value_for_type_at_offset(msg, info->type, offset, ret);

    return true;
}


template<typename R>
inline void MsgHandler::field_value_for_type_at_offset(const uint8_t *msg,
                                                       uint8_t type,
                                                       uint8_t offset,
                                                       R &ret)
{
    _field_value_for_type_at_offset(msg, type, offset, ret);
}

// handle bool return case specially so we can use is_zero on floats:
inline void MsgHandler::field_value_for_type_at_offset(const uint8_t *msg,
                                                       uint8_t type,
                                                       uint8_t offset,
                                                       bool &ret)
{
    switch (type) {
    case 'f': {
        float value = 0;
        const uint8_t to_copy = size_for_type(type);
        memcpy((void *)&value, &msg[offset], to_copy);
        ret = ! is_zero(value);
        break;
    }
    default:
        _field_value_for_type_at_offset(msg, type, offset, ret);
    }
}

template<typename R>
inline void MsgHandler::_field_value_for_type_at_offset(const uint8_t *msg,
                                                        const uint8_t type,
                                                        const uint8_t offset,
                                                        R &ret)
{
    uint8_t to_copy = size_for_type(type);
    union {
        uint8_t u8;
        int16_t i16;
        int32_t i32;
        int64_t i64;
        uint16_t u16;
        uint32_t u32;
        uint64_t u64;
        float f;
    } dest;

    memset(&dest, 0, sizeof(dest));

    if (sizeof(R) < to_copy) {
            // we should never ask to e.g. store a uint16 from a msg into e.g. a uint8
	::fprintf(stderr, "Internal error: destination too small (want=%u have=%u) type=%u\n", (unsigned)to_copy, (unsigned)sizeof(R), (unsigned)type);
	abort();
    }
    memcpy(&dest, &msg[offset], to_copy);

  /* we register the types - add_field_type - so can we do without
     * this switch statement somehow? */
    switch (type) {
    case 'M':
    case 'B':
        ret = (R)dest.u8;
        break;
    case 'c':
    case 'h':
        ret = (R)dest.i16;
        break;
    case 'H':
        ret = (R)dest.u16;
        break;
    case 'C':
        ret = (R)dest.u16;
        break;
    case 'f':
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
        ret = (R)dest.f;
#pragma GCC diagnostic pop
        break;
    case 'I':
    case 'E':
        ret = (R)dest.u32;
        break;
    case 'L':
    case 'e':
        ret = (R)dest.i32;
        break;
    case 'q':
        ret = (R)dest.i64;
        break;
    case 'Q':
        ret = (R)dest.u64;
        break;
    default:
        ::printf("Unhandled format type (%c)\n", type);
        ::abort();
    }
}

#endif
