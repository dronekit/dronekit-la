#include "dataflash_textdump_reader.h"

#include <vector>
#include <string>

#include "stdlib.h"
#include "string.h"

#include "tools.h"

// BT 42 EN
std::vector<std::string> split_line(const uint8_t *line, uint32_t len)
{
    std::vector<std::string> ret;

    std::string source = std::string((const char*)line, len);
    while (true) {
        int32_t pos = source.find(", ");
        if (pos == -1) {
            break;
        }
        ret.push_back(source.substr(0,pos));
        source = source.substr(pos+2);
    }
    ret.push_back(source);

    return ret;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
// is everything we do here kosher?  Can we really legally assign a
// float into an un-aligned memory location?

void DataFlash_TextDump_Reader::handle_line(const uint8_t *line, uint32_t len)
{
    std::vector<std::string> items = split_line(line, len);
    // fprintf(stderr, "Got %s\n", items[0].c_str());
    const uint8_t TYPESTRING = 0;
    if (items[TYPESTRING] == "FMT") {
        const uint8_t TYPENUM = 1;
        const uint8_t TYPELEN = 2;
        const uint8_t TYPENAME = 3;
        const uint8_t TYPEFORMAT = 4;
        const uint8_t TYPELABELS = 5;
        const uint8_t defining_type = atoi(items[TYPENUM].c_str());

        struct log_Format &f = formats[defining_type];
        f.type = defining_type;
        f.length = atoi(items[TYPELEN].c_str());
        memset(f.name, '\0', sizeof(f.name));
        memset(f.format, '\0', sizeof(f.format));
        memset(f.labels, '\0', sizeof(f.labels));
        memcpy(f.name, items[TYPENAME].c_str(), strlen(items[TYPENAME].c_str()));
        memcpy(f.format, items[TYPEFORMAT].c_str(), strlen(items[TYPEFORMAT].c_str()));
        memcpy(f.labels, items[TYPELABELS].c_str(), strlen(items[TYPELABELS].c_str()));
        typename_to_typenum[items[TYPENAME]] = defining_type;

        handle_format_message_received(items[TYPENAME].c_str(), f, (const char*)&f);

        return;
    }

    if (typename_to_typenum.count(items[TYPESTRING]) == 0) {
        ::fprintf(stderr, "no FMT message received for (%s)\n", items[TYPESTRING].c_str());
        return;
    }

    const uint8_t typenum = typename_to_typenum[items[TYPESTRING]];
    struct log_Format &f = formats[typenum];
    uint8_t buf[1024]; // nothing out there is this big....
    char format_types[256]; // no format  is this long
    memset(format_types, '\0', sizeof(format_types));
    memcpy(format_types, f.format, sizeof(f.format));
    uint8_t buf_offset = 0;
    buf[buf_offset++] = HEAD_BYTE1;
    buf[buf_offset++] = HEAD_BYTE2;
    buf[buf_offset++] = typenum;
    for (uint8_t i=0; i< strlen(format_types); i++) {
        char format_char = format_types[i];
        uint64_t value_integer = atoi(items[1+i].c_str());
        float value_float = atof(items[1+i].c_str());
        int32_t xvalue = value_float * 10000000.0f;
        switch(format_char) {
        case 'f':
            *((float*)&buf[buf_offset]) = value_float;
            buf_offset += sizeof(float);
            break;
        case 'd':
            *((double*)&buf[buf_offset]) = value_float;
            buf_offset += sizeof(double);
            break;
        case 'E':
            value_integer *= 100;
            FALLTHROUGH;
        case 'I':
            *((uint32_t*)&buf[buf_offset]) = value_integer; // FIXME
            buf_offset += sizeof(uint32_t);
            break;
        case 'L':
            *((int32_t*)&buf[buf_offset]) = xvalue;
            buf_offset += sizeof(int32_t);
            break;
        case 'e':
            value_integer *= 100;
            FALLTHROUGH;
        case 'i':
            *((int32_t*)&buf[buf_offset]) = value_integer; // FIXME
            buf_offset += sizeof(int32_t);
            break;
        case 'c':
            value_integer *= 100;
            FALLTHROUGH;
        case 'h':
            *((int16_t*)&buf[buf_offset]) = value_integer; // FIXME
            buf_offset += sizeof(int16_t);
            break;
        case 'C':
            value_integer *= 100;
            FALLTHROUGH;
        case 'H':
            *((uint16_t*)&buf[buf_offset]) = value_integer; // FIXME
            buf_offset += sizeof(uint16_t);
            break;
        case 'B':
            FALLTHROUGH;
        case 'M':
            *((uint8_t*)&buf[buf_offset]) = value_integer; // FIXME
            buf_offset += sizeof(uint8_t);
            break;
        case 'b':
            *((int8_t*)&buf[buf_offset]) = value_integer; // FIXME
            buf_offset += sizeof(int8_t);
            break;
        case 'q':
            *((int64_t*)&buf[buf_offset]) = value_integer; // FIXME
            buf_offset += sizeof(int64_t);
            break;
        case 'Q':
            *((uint64_t*)&buf[buf_offset]) = value_integer; // FIXME
            buf_offset += sizeof(uint64_t);
            break;
        case 'N':
            strncpy(((char*)&buf[buf_offset]), items[1+i].c_str(), 16); // FIXME
            buf_offset += 16;
            break;
        case 'Z':
            strncpy(((char*)&buf[buf_offset]), items[1+i].c_str(), 64); // FIXME
            buf_offset += 64;
            break;
        default:
            fprintf(stderr, "Unknown format char %c\n", format_char);
            abort();
        }
    }
    if (buf_offset != f.length) {
        ::fprintf(stderr, "Error packing message: %d != %d\n", buf_offset, f.length);
        abort();
    }
    handle_message_received(f, buf);
}
#pragma GCC diagnostic pop

uint32_t DataFlash_TextDump_Reader::feed(const uint8_t *buf, const uint32_t len)
{
    ssize_t total_bytes_used = 0;
    // ::fprintf(stderr, "feed (%u bytes)\n", len);
    ssize_t end_of_line_pointer = 0;
    while (len - end_of_line_pointer >= 1) {
        uint8_t line_ending_length = 0;
        if (len - end_of_line_pointer >= 2 &&
            buf[end_of_line_pointer] == '\r' && buf[end_of_line_pointer+1] == '\n') {
            line_ending_length = 2;
        } else if (buf[end_of_line_pointer] == '\n') {
            line_ending_length = 1;
        }

        // deal with stuff between total_bytes_used and end_of_line_pointer
        if (line_ending_length) {
            handle_line(&buf[total_bytes_used], end_of_line_pointer-total_bytes_used);
            end_of_line_pointer += line_ending_length;
            total_bytes_used = end_of_line_pointer;
        } else {
            end_of_line_pointer++;
        }
    }

    return total_bytes_used;
}
