#ifndef _MAVLINK_WRITER_H
#define _MAVLINK_WRITER_H

#include <vector>

#pragma GCC diagnostic push
#if defined(__GNUC__) && __GNUC__ >= 9
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#endif
#pragma GCC diagnostic ignored "-Wpedantic"
#include "mavlink_c_library/ardupilotmega/mavlink.h"
#pragma GCC diagnostic pop

#include "INIReader.h"
#include "telem_client.h"

#define UNUSED __attribute__ ((unused))

class MAVLink_Writer {
public:
    MAVLink_Writer(INIReader *config UNUSED)
        { }
    void send_message(const mavlink_message_t &msg);

    void add_client(Telem_Client *);
    bool any_data_to_send();

private:
    std::vector<Telem_Client *> clients;

};

#endif
