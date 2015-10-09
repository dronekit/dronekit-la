#include "mavlink_writer.h"

bool MAVLink_Writer::handle_message(const mavlink_message_t &msg)
{
    char sendbuf[1024]; // large enough...

    uint16_t messageLen = mavlink_msg_to_send_buffer((uint8_t*)sendbuf,&msg);
    if (send_buffer_space_free() < messageLen) {
        // dropped_packets++;
        return false;
    }
    if (_buf_stop >= _buf_start) {
        uint16_t to_copy = _buf_stop - _buf_start;
        memcpy(&sendbuf[_buf_start], sendbuf, to_copy);
    } else {
        uint16_t to_copy = _buf_len - _buf_start;
        memcpy(&sendbuf[_buf_start], sendbuf, to_copy);
        to_copy = messageLen - to_copy;
        memcpy(&sendbuf[0], sendbuf, to_copy);
    }
    return true;
}


