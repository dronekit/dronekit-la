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
        uint16_t to_copy = _buf_len - _buf_stop;
        if (to_copy > messageLen) {
            to_copy = messageLen;
        }
        memcpy(&_buf[_buf_stop], sendbuf, to_copy);
        _buf_stop += to_copy;
        if (_buf_stop >= _buf_len) {
            _buf_stop = 0;
        }
        to_copy = messageLen - to_copy;
        if (to_copy) {
            memcpy(&_buf[_buf_stop], &sendbuf[messageLen-to_copy], to_copy);
            _buf_stop += to_copy;
            if (_buf_stop >= _buf_len) {
                _buf_stop = 0;
            }
        }
    } else {
        memcpy(&_buf[_buf_stop], &sendbuf[0], messageLen);
        _buf_stop += messageLen;
        if (_buf_stop >= _buf_len) {
            _buf_stop = 0;
        }
    }
    return true;
}


