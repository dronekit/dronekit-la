#ifndef _DATAFLASH_TEXTDUMP_READER_H
#define _DATAFLASH_TEXTDUMP_READER_H

#include "format_reader.h"
#include "dataflash_reader.h"

/* encapsulates a DataFlash_Reader object; turns the text log back
 * into a binary log... */

class DataFlash_TextDump_Reader : public DataFlash_Reader {
public:
    DataFlash_TextDump_Reader(INIReader *config) :
        DataFlash_Reader(config)
        { }

    uint32_t feed(const uint8_t *buf, const uint32_t len) override;

private:
    void handle_line(const uint8_t *line, uint32_t len);

    std::map<std::string,uint8_t> typename_to_typenum;
};

#endif
