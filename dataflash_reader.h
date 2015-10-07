#ifndef _DATAFLASH_READER_H
#define _DATAFLASH_READER_H

#include "format_reader.h"

#define PACKED

#include "DataFlash/LogMessage.h"
#include "Vector3f.h"
#include "Location.h"

#include "dataflash_message_handler.h"

class DataFlash_Reader : public Format_Reader {
public:
    DataFlash_Reader(INIReader *config) :
        Format_Reader(config)
        { }

    uint32_t feed(const uint8_t *buf, const uint32_t len) override;

protected:
    ssize_t count_bytes_skipped = 0;
#define MAX_FORMATS 256
    struct log_Format formats[MAX_FORMATS] = { };
    char format_names[MAX_FORMATS][5] = { };

    void handle_format_message_received(const char *name, const struct log_Format &format, const char *msg);
    void handle_message_received(const struct log_Format &format, const uint8_t *msg);
    uint32_t packet_count = 0;

    void end_of_log() override;
};

#endif
