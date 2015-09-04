#ifndef MAVLINK_READER_H
#define MAVLINK_READER_H

#include <stdint.h>
#include <netinet/in.h>

#include "INIReader.h"
#include "util.h"

#include "mavlink_message_handler.h"
#include "analyze.h" // for output_style_option

#include "format_reader.h"

/* A mavlink packet should be limited to 6+255+2 = 263 bytes
   6 byte header, 255 max bytes in payload, 2 byte crc */
#define TELEM_PKT_MAX 512

class MAVLink_Reader : public Format_Reader {
public:
    MAVLink_Reader(INIReader *config) :
        Format_Reader(config),
	err_skipped(0),
	err_time_us(0),            /* last time we logged */
	err_interval_us(1000000),  /* once per second max */
        _is_tlog(false)
    { }

    uint32_t feed(const uint8_t *buf, const uint32_t len) override;

    void set_is_tlog(bool value) { _is_tlog = value; }
    bool is_tlog() { return _is_tlog; }

protected:
    void end_of_log() override;

private:
    int can_log_error();

    Analyze::output_style_option output_style;
    
    void handle_message_received(uint64_t timestamp, mavlink_message_t msg);

    template <typename msgtype>
    void handle_decoded_message_received(uint64_t timestamp, msgtype &msg) {
        for(int i=0; i<next_message_handler; i++) {
            ((MAVLink_Message_Handler*)message_handler[i])->handle_decoded_message(timestamp, msg);
        }
    }

    void handle_packet_received(uint8_t *pkt, uint16_t size);

    uint16_t err_skipped;
    uint64_t err_time_us;
    uint64_t err_interval_us;

    uint32_t packet_count = 0;
    uint8_t timestamp_offset = 0;
    bool done_timestamp = false;
    uint64_t timestamp = 0;
    mavlink_message_t mav_msg;
    mavlink_status_t mav_status;

    bool _is_tlog;
};


#endif
