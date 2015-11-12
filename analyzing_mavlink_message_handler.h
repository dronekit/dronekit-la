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
        _analyze->add_data_source("ALTITUDE_ESTIMATE_AHRS2", "AHRS2.alt");
        _analyze->add_data_source("ALTITUDE_ESTIMATE_GLOBAL_POSITION_INT", "GLOBAL_POSITION_INT.alt");
        _analyze->add_data_source("ALTITUDE_ESTIMATE_GPS_RAW_INT", "GPS_RAW_INT.alt");

        _analyze->add_data_source("ALTITUDE_ESTIMATE_SCALED_PRESSURE", "SCALED_PRESSURE.temperature");
        _analyze->add_data_source("ALTITUDE_ESTIMATE_SCALED_PRESSURE", "SCALED_PRESSURE.press_abs");
        _analyze->add_data_source("ALTITUDE_ESTIMATE_SCALED_PRESSURE2", "SCALED_PRESSURE2.temperature");
        _analyze->add_data_source("ALTITUDE_ESTIMATE_SCALED_PRESSURE2", "SCALED_PRESSURE2.press_abs");

        _analyze->add_data_source("ARMING", "HEARTBEAT.base_mode");

        _analyze->add_data_source("ATTITUDE", "ATTITUDE.roll");
        _analyze->add_data_source("ATTITUDE", "ATTITUDE.pitch");
        _analyze->add_data_source("ATTITUDE", "ATTITUDE.yaw");
        _analyze->add_data_source("ATTITUDE_ESTIMATE_ATTITUDE", "ATTITUDE.roll");
        _analyze->add_data_source("ATTITUDE_ESTIMATE_ATTITUDE", "ATTITUDE.pitch");
        _analyze->add_data_source("ATTITUDE_ESTIMATE_ATTITUDE", "ATTITUDE.yaw");
        _analyze->add_data_source("ATTITUDE_ESTIMATE_AHRS2", "AHRS2.roll");
        _analyze->add_data_source("ATTITUDE_ESTIMATE_AHRS2", "AHRS2.pitch");
        _analyze->add_data_source("ATTITUDE_ESTIMATE_AHRS2", "AHRS2.yaw");

        _analyze->add_data_source("AUTOPILOT_SCHEDULING", "STATUSTEXT.text");
        
        _analyze->add_data_source("BATTERY_REMAINING", "SYS_STATUS.battery_remaining");

        _analyze->add_data_source("CRASHED", "HEARTBEAT.system_status");

        _analyze->add_data_source("DESATTITUDE", "NAV_CONTROLLER_OUTPUT.nav_roll");
        _analyze->add_data_source("DESATTITUDE", "NAV_CONTROLLER_OUTPUT.nav_pitch");
        _analyze->add_data_source("DESATTITUDE", "NAV_CONTROLLER_OUTPUT.nav_bearing");

        _analyze->add_data_source("EKF_FLAGS", "EKF_STATUS_REPORT.flags");
        _analyze->add_data_source("EKF_VARIANCES_velocity_variance", "EKF_STATUS_REPORT.velocity");
        _analyze->add_data_source("EKF_VARIANCES_pos_horiz_variance", "EKF_STATUS_REPORT.pos_horiz");
        _analyze->add_data_source("EKF_VARIANCES_pos_vert_variance", "EKF_STATUS_REPORT.pos_vert");
        _analyze->add_data_source("EKF_VARIANCES_compass_variance", "EKF_STATUS_REPORT.compass");
        _analyze->add_data_source("EKF_VARIANCES_terrain_alt_variance", "EKF_STATUS_REPORT.terrain_alt");

        _analyze->add_data_source("GPSINFO_GPS_RAW_INT", "GPS_RAW_INT.satellites_visible");
        _analyze->add_data_source("GPSINFO_GPS_RAW_INT", "GPS_RAW_INT.eph");
        _analyze->add_data_source("GPSINFO_FIXTYPE_GPS_RAW_INT", "GPS_RAW_INT.fix_type");
        _analyze->add_data_source("GPSINFO_FIXTYPE_GPS_RAW_INT", "SYSTEM_TIME.time_boot_ms");

        _analyze->add_data_source("SENSORS_HEALTH", "SYS_STATUS.onboard_control_sensors_present");
        _analyze->add_data_source("SENSORS_HEALTH", "SYS_STATUS.onboard_control_sensors_enabled");
        _analyze->add_data_source("SENSORS_HEALTH", "SYS_STATUS.onboard_control_sensors_health");

        _analyze->add_data_source("SERVO_OUTPUT", "SERVO_OUTPUT_RAW.servo1_raw");
        _analyze->add_data_source("SERVO_OUTPUT", "SERVO_OUTPUT_RAW.servo2_raw");
        _analyze->add_data_source("SERVO_OUTPUT", "SERVO_OUTPUT_RAW.servo3_raw");
        _analyze->add_data_source("SERVO_OUTPUT", "SERVO_OUTPUT_RAW.servo4_raw");
        _analyze->add_data_source("SERVO_OUTPUT", "SERVO_OUTPUT_RAW.servo5_raw");
        _analyze->add_data_source("SERVO_OUTPUT", "SERVO_OUTPUT_RAW.servo6_raw");
        _analyze->add_data_source("SERVO_OUTPUT", "SERVO_OUTPUT_RAW.servo7_raw");
        _analyze->add_data_source("SERVO_OUTPUT", "SERVO_OUTPUT_RAW.servo8_raw");

        _analyze->add_data_source("ORIGIN", "AHRS2.lat");
        _analyze->add_data_source("ORIGIN", "AHRS2.lng");
        _analyze->add_data_source("ORIGIN", "AHRS2.alt");

        _analyze->add_data_source("PARAM", "PARAM.param_id");
        _analyze->add_data_source("PARAM", "PARAM.value");

        _analyze->add_data_source("POSITION", "GLOBAL_POSITION_INT.lat");
        _analyze->add_data_source("POSITION", "GLOBAL_POSITION_INT.lon");

        _analyze->add_data_source("POSITION_ESTIMATE_AHRS2", "AHRS2.lat");
        _analyze->add_data_source("POSITION_ESTIMATE_AHRS2", "AHRS2.lng");

        _analyze->add_data_source("POSITION_ESTIMATE_GPS_RAW_INT", "GPS_RAW_INT.lat");
        _analyze->add_data_source("POSITION_ESTIMATE_GPS_RAW_INT", "GPS_RAW_INT.lon");
        
        _analyze->add_data_source("VELOCITY_GROUND", "GLOBAL_POSITION_INT.vx");
        _analyze->add_data_source("VELOCITY_GROUND", "GLOBAL_POSITION_INT.vy");
        _analyze->add_data_source("VELOCITY_GROUND", "GLOBAL_POSITION_INT.vz");

        _analyze->add_data_source("VEHICLE_DEFINITION", "STATUSTEXT.text");
        _analyze->add_data_source("SYSTEM_TIME", "SYSTEM_TIME.boot_time_ms");
    }

private:
    virtual void handle_decoded_message_scaled_pressure(uint64_t T, const char *name, double press_abs, double temperature);

    virtual void handle_decoded_message(uint64_t T, mavlink_ahrs2_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_attitude_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_ekf_status_report_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_gps_raw_int_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_global_position_int_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_heartbeat_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_nav_controller_output_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_param_value_t &msg) override;    
    virtual void handle_decoded_message(uint64_t T, mavlink_scaled_pressure_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_scaled_pressure2_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_servo_output_raw_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_statustext_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_sys_status_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_system_time_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_vfr_hud_t &msg) override;

    void end_of_log(uint32_t packet_count, uint64_t bytes_dropped = 0) override;
    
    Analyze *_analyze;
    AnalyzerVehicle::Base *&_vehicle;

    void set_vehicle_copter();

    // many of these aren't sensors...
    const std::map<const std::string, const uint64_t> _sensor_masks = {
        { "MAV_SYS_STATUS_SENSOR_3D_GYRO", 1 },  /* 0x01 3D gyro | */
	{ "MAV_SYS_STATUS_SENSOR_3D_ACCEL", 2 },  /* 0x02 3D accelerometer | */
	{ "MAV_SYS_STATUS_SENSOR_3D_MAG", 4 },  /* 0x04 3D magnetometer | */
	{ "MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE", 8 },  /* 0x08 absolute pressure | */
	{ "MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE", 16 },  /* 0x10 differential pressure | */
	{ "MAV_SYS_STATUS_SENSOR_GPS", 32 },  /* 0x20 GPS | */
	{ "MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW", 64 },  /* 0x40 optical flow | */
	{ "MAV_SYS_STATUS_SENSOR_VISION_POSITION", 128 },  /* 0x80 computer vision position | */
	{ "MAV_SYS_STATUS_SENSOR_LASER_POSITION", 256 },  /* 0x100 laser based position | */
	{ "MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH", 512 },  /* 0x200 external ground truth (Vicon or Leica) | */
	{ "MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL", 1024 },  /* 0x400 3D angular rate control | */
	{ "MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION", 2048 },  /* 0x800 attitude stabilization | */
	{ "MAV_SYS_STATUS_SENSOR_YAW_POSITION", 4096 },  /* 0x1000 yaw position | */
	{ "MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL", 8192 },  /* 0x2000 z/altitude control | */
	{ "MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL", 16384 },  /* 0x4000 x/y position control | */
	{ "MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS", 32768 },  /* 0x8000 motor outputs / control | */
	{ "MAV_SYS_STATUS_SENSOR_RC_RECEIVER", 65536 },  /* 0x10000 rc receiver | */
	{ "MAV_SYS_STATUS_SENSOR_3D_GYRO2", 131072 },  /* 0x20000 2nd 3D gyro | */
	{ "MAV_SYS_STATUS_SENSOR_3D_ACCEL2", 262144 },  /* 0x40000 2nd 3D accelerometer | */
	{ "MAV_SYS_STATUS_SENSOR_3D_MAG2", 524288 },  /* 0x80000 2nd 3D magnetometer | */
	{ "MAV_SYS_STATUS_GEOFENCE", 1048576 },  /* 0x100000 geofence | */
	{ "MAV_SYS_STATUS_AHRS", 2097152 },  /* 0x200000 AHRS subsystem health | */
	{ "MAV_SYS_STATUS_TERRAIN", 4194304 },  /* 0x400000 Terrain subsystem health | */
	{ "MAV_SYS_STATUS_SENSOR_ENUM_END", 4194305 }  /*  | */
    };

    bool set_origin_was_armed = false;
};


#endif
