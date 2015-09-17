#ifndef _ANALYZING_MAVLINK_MESSAGE_HANDLER_H
#define _ANALYZING_MAVLINK_MESSAGE_HANDLER_H

#include "mavlink_message_handler.h"
#include "analyze.h"

class Analyzing_MAVLink_Message_Handler : public MAVLink_Message_Handler {
public:
    Analyzing_MAVLink_Message_Handler(Analyze *analyze,
                                      AnalyzerVehicle::Base *&vehicle) :
        MAVLink_Message_Handler(),
        _analyze(analyze),
        _vehicle(vehicle) {

        _analyze->add_data_source("ALTITUDE", "GLOBAL_POSITION_INT.alt");

        _analyze->add_data_source("ARMING", "HEARBEAT.base_mode");

        _analyze->add_data_source("ATTITUDE", "ATTITUDE.roll");
        _analyze->add_data_source("ATTITUDE", "ATTITUDE.pitch");
        _analyze->add_data_source("ATTITUDE", "ATTITUDE.yaw");
        _analyze->add_data_source("ATTITUDE_ESTIMATE_ATTITUDE", "ATTITUDE.roll");
        _analyze->add_data_source("ATTITUDE_ESTIMATE_ATTITUDE", "ATTITUDE.pitch");
        _analyze->add_data_source("ATTITUDE_ESTIMATE_ATTITUDE", "ATTITUDE.yaw");
        _analyze->add_data_source("ATTITUDE_ESTIMATE_AHRS2", "AHRS2.roll");
        _analyze->add_data_source("ATTITUDE_ESTIMATE_AHRS2", "AHRS2.pitch");
        _analyze->add_data_source("ATTITUDE_ESTIMATE_AHRS2", "AHRS2.yaw");

        
        _analyze->add_data_source("BATTERY_REMAINING", "SYS_STATUS.battery_remaining");

        _analyze->add_data_source("DESATTITUDE", "NAV_CONTROLLER_OUTPUT.nav_roll");
        _analyze->add_data_source("DESATTITUDE", "NAV_CONTROLLER_OUTPUT.nav_pitch");
        _analyze->add_data_source("DESATTITUDE", "NAV_CONTROLLER_OUTPUT.nav_bearing");

        _analyze->add_data_source("EKF_FLAGS", "EKF_STATUS_REPORT.flags");
        _analyze->add_data_source("EKF_VARIANCE_velocity_variance", "EKF_STATUS_REPORT.velocity");
        _analyze->add_data_source("EKF_VARIANCE_pos_horiz_variance", "EKF_STATUS_REPORT.pos_horiz");
        _analyze->add_data_source("EKF_VARIANCES_pos_vert_variance", "EKF_STATUS_REPORT.pos_vert");
        _analyze->add_data_source("EKF_VARIANCES_compass_variance", "EKF_STATUS_REPORT.compass");
        _analyze->add_data_source("EKF_VARIANCES_terrain_alt_variance", "EKF_STATUS_REPORT.terrain_alt");

        _analyze->add_data_source("SERVO_OUTPUT", "SERVO_OUTPUT_RAW.servo1_raw");
        _analyze->add_data_source("SERVO_OUTPUT", "SERVO_OUTPUT_RAW.servo2_raw");
        _analyze->add_data_source("SERVO_OUTPUT", "SERVO_OUTPUT_RAW.servo3_raw");
        _analyze->add_data_source("SERVO_OUTPUT", "SERVO_OUTPUT_RAW.servo4_raw");
        _analyze->add_data_source("SERVO_OUTPUT", "SERVO_OUTPUT_RAW.servo5_raw");
        _analyze->add_data_source("SERVO_OUTPUT", "SERVO_OUTPUT_RAW.servo6_raw");
        _analyze->add_data_source("SERVO_OUTPUT", "SERVO_OUTPUT_RAW.servo7_raw");
        _analyze->add_data_source("SERVO_OUTPUT", "SERVO_OUTPUT_RAW.servo8_raw");

        _analyze->add_data_source("PARAM", "PARAM.param_id");
        _analyze->add_data_source("PARAM", "PARAM.value");

        _analyze->add_data_source("POSITION", "GLOBAL_POSITION_INT.lat");
        _analyze->add_data_source("POSITION", "GLOBAL_POSITION_INT.lon");

        _analyze->add_data_source("POSITION_ESTIMATE_AHRS2", "AHRS2.lat");
        _analyze->add_data_source("POSITION_ESTIMATE_AHRS2", "AHRS2.lng");

        _analyze->add_data_source("POSITION_ESTIMATE_GPS_RAW_INT", "GPS_RAW_INT.lat");
        _analyze->add_data_source("POSITION_ESTIMATE_GPS_RAW_INT", "GPS_RAW_INT.lon");
        
        _analyze->add_data_source("VEHICLE_DEFINITION", "STATUSTEXT.text");
    }

private:
    virtual void handle_decoded_message(uint64_t T, mavlink_ahrs2_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_attitude_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_ekf_status_report_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_gps_raw_int_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_global_position_int_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_heartbeat_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_nav_controller_output_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_param_value_t &msg) override;    
    virtual void handle_decoded_message(uint64_t T, mavlink_servo_output_raw_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_statustext_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_sys_status_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_vfr_hud_t &msg) override;

    void end_of_log(uint32_t packet_count) override;
    
    Analyze *_analyze;
    AnalyzerVehicle::Base *&_vehicle;

    void set_vehicle_copter();
};


#endif
