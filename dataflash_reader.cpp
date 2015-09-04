#include "dataflash_reader.h"

#include "LA_MsgHandler.h"

#include <string.h>

void DataFlash_Reader::handle_format_message_received(const char *name, const struct log_Format &format, const char *msg)
{
    for(int i=0; i<next_message_handler; i++) {
        ((DataFlash_Message_Handler*)message_handler[i])->handle_format_message_received(name, format, msg);
    }
}

void DataFlash_Reader::handle_message_received(const struct log_Format &format, const uint8_t *msg)
{
    for(int i=0; i<next_message_handler; i++) {
        ((DataFlash_Message_Handler*)message_handler[i])->handle_message_received(format, msg);
    }
}

#define LOG_FORMAT_MSG 128
uint32_t DataFlash_Reader::feed(const uint8_t *buf, const uint32_t len)
{
    ssize_t total_bytes_used = 0;
    // ::fprintf(stderr, "feed (%u bytes)\n", len);
    while (true) {
        if (len - total_bytes_used < 3) {
            return total_bytes_used;
        }

        // skip through input until we find a valid header:
        if (buf[total_bytes_used] != HEAD_BYTE1 || buf[total_bytes_used+1] != HEAD_BYTE2) {
            total_bytes_used += 1;
            count_bytes_skipped++;
            continue;
        }

        uint8_t type = buf[total_bytes_used+2];
        if (type == LOG_FORMAT_MSG) {
            // fprintf(stderr, "Found LOG_FORMAT_MESSAGE\n");
            if ((uint32_t)(len-total_bytes_used) < sizeof(struct log_Format)) {
                return total_bytes_used;
            }
            uint8_t defining_type = ((struct log_Format*)(&(buf[total_bytes_used])))->type;
            struct log_Format &f = formats[defining_type];
            memcpy(&f, &buf[total_bytes_used], sizeof(struct log_Format));
            for (uint8_t i=0;i<4;i++) {
                if (!isascii(f.name[i]) &&
                    !f.name[i] == '\0') {
                    // name is assumed to be ascii; it looked like a
                    // format message, but wasn't.
                    total_bytes_used++;
                    count_bytes_skipped++;
                    goto SKIP;
                }
            }
            memcpy(format_names[defining_type], f.name, 4);
            // ::fprintf(stderr, "Format %u = %s  length=%u\n", f.type, format_names[defining_type], f.length);
            packet_count++;
            handle_format_message_received(format_names[defining_type], f, (const char*)&buf[total_bytes_used]);
            total_bytes_used += sizeof(struct log_Format);
            // memcpy(&formats[f.type], &f, sizeof(formats[f.type]));
        SKIP:
            continue;
        }
        const struct log_Format &format = formats[type];
        if (format.length == 0) {
            // could do this, but hasn't happened in practice:
            // total_bytes_used += 1;
            // count_bytes_skipped++;
            // continue;
            ::fprintf(stderr, "no FMT message received for type=%u\n", type);
            abort();
        }

        if (len - total_bytes_used < format.length) {
            return total_bytes_used;
        }
        // ::fprintf(stderr, "Received message of type %u\n", type);
        handle_message_received(format, &buf[total_bytes_used]);
        packet_count++;
        total_bytes_used += format.length;
    }
    ::fprintf(stderr, "Should not be reached\n");
    abort();
}

void DataFlash_Reader::end_of_log()
{
    for(int i=0; i<next_message_handler; i++) {
        message_handler[i]->end_of_log(packet_count);
    }
}
