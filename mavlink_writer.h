#ifndef _MAVLINK_WRITER_H
#define _MAVLINK_WRITER_H

#include "mavlink/c_library/ardupilotmega/mavlink.h"
#include "INIReader.h"

#define UNUSED __attribute__ ((unused))

class MAVLink_Writer {
public:

    MAVLink_Writer(INIReader *config UNUSED, uint8_t *buf, const uint32_t buf_len,
                   uint32_t &buf_start, uint32_t &buf_stop) :
        _buf(buf),
        _buf_len(buf_len),
        _buf_start(buf_start),
        _buf_stop(buf_stop)
        { }
    bool handle_message(const mavlink_message_t &msg);

    // virtual void handle_message(mavlink_ahrs2_t &msg) { }
    // virtual void handle_message(mavlink_attitude_t &msg) { }
    // virtual void handle_message(mavlink_ekf_status_report_t &msg) { }
    // virtual void handle_message(mavlink_gps_raw_int_t &msg) { }
    // virtual void handle_message(mavlink_global_position_int_t &msg) { }
    // virtual void handle_message(mavlink_heartbeat_t &msg) { }
    // virtual void handle_message(mavlink_mount_status_t &msg) { }
    // virtual void handle_message(mavlink_nav_controller_output_t &msg) { }
    // virtual void handle_message(mavlink_param_value_t &msg) { }
    // virtual void handle_message(mavlink_remote_log_data_block_t &msg) { }
    // virtual void handle_message(mavlink_scaled_pressure_t &msg) { }
    // virtual void handle_message(mavlink_scaled_pressure2_t &msg) { }
    // virtual void handle_message(mavlink_servo_output_raw_t &msg) { }
    // virtual void handle_message(mavlink_sys_status_t &msg) { }
    // virtual void handle_message(mavlink_statustext_t &msg) { }
    // virtual void handle_message(mavlink_vfr_hud_t &msg) { }

    uint8_t buf_used() {
        return _buf_len - send_buffer_space_free();
    }
private:

    uint8_t *_buf;
    uint32_t _buf_len = 0;
    uint32_t &_buf_start;
    uint32_t &_buf_stop;
    
    uint32_t send_buffer_space_free() {
        if (_buf_stop > _buf_start) {
            return _buf_stop - _buf_start;
        }
        return _buf_stop + _buf_len - _buf_start;
    };
};

#endif
