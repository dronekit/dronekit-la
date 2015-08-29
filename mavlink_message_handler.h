#ifndef MAVLINK_MESSAGE_HANDLER_H
#define MAVLINK_MESSAGE_HANDLER_H

#include <stdio.h>

/*
 * mavlink_message_handler
 *
 * A base class for objects which process mavlink messages and
 * possibly send responses
 *
 */

#include <stdint.h>

#include "INIReader.h"
#include "mavlink/c_library/ardupilotmega/mavlink.h"
#include "mavlink/c_library/common/mavlink.h"

#include <netinet/in.h>

class MAVLink_Message_Handler {
public:
    MAVLink_Message_Handler(int fd, struct sockaddr_in *sa) :
	_fd_telem_forwarder(fd),
	_sa_telemetry_forwarder(sa)
    { }

    bool configure(INIReader *config) {
	system_id = config->GetInteger("dflogger", "system_id", 254);
	component_id = config->GetInteger("dflogger", "component_id", 17);
	return true;
    }

    virtual void idle_tenthHz() { }
    virtual void idle_1Hz() { }
    virtual void idle_10Hz() { }
    virtual void idle_100Hz() { }

    virtual void sighup_received() { }

    virtual void handle_packet(uint8_t *pkt, uint16_t pktlen) { }

    // template <typename msgtype>
    // virtual void handle_decoded_message(msgtype &msg) { }

    virtual void handle_decoded_message(uint64_t T, mavlink_ahrs2_t &msg) { }
    virtual void handle_decoded_message(uint64_t T, mavlink_attitude_t &msg) { }
    virtual void handle_decoded_message(uint64_t T, mavlink_ekf_status_report_t &msg) { }
    virtual void handle_decoded_message(uint64_t T, mavlink_heartbeat_t &msg) { }
    virtual void handle_decoded_message(uint64_t T, mavlink_nav_controller_output_t &msg) { }
    virtual void handle_decoded_message(uint64_t T, mavlink_param_value_t &msg) { }
    virtual void handle_decoded_message(uint64_t T, mavlink_remote_log_data_block_t &msg) { }
    virtual void handle_decoded_message(uint64_t T, mavlink_servo_output_raw_t &msg) { }
    virtual void handle_decoded_message(uint64_t T, mavlink_sys_status_t &msg) { }
    virtual void handle_decoded_message(uint64_t T, mavlink_statustext_t &msg) { }
    virtual void handle_decoded_message(uint64_t T, mavlink_vfr_hud_t &msg) { }

    virtual void end_of_log(uint32_t packet_count) { }

protected:
    void send_message_to_telem_forwarder(mavlink_message_t &msg);
    uint8_t system_id;
    uint8_t component_id;

    int _fd_telem_forwarder;
    struct sockaddr_in *_sa_telemetry_forwarder;
private:
};

#endif
