#ifndef HEART_H
#define HEART_H

/*
 * heart
 *
 * Periodically send mavlink heartbeats to our upstream connection
 *
 */

#include "mavlink_message_handler.h"

class Heart : public MAVLink_Message_Handler {

public:
    Heart(int fd, struct sockaddr_in *sa) :
	MAVLink_Message_Handler(fd, sa),
	last_heartbeat_time(0),
	heartbeat_interval(5000000) // microseconds
    { }

    void idle_10Hz();
    bool configure(INIReader *config);

private:
    uint64_t last_heartbeat_time;
    const uint32_t heartbeat_interval;
    void beat();
};

#endif
