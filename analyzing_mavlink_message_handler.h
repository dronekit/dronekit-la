/**
 * @file
 * @author Peter Barker <peter.barker@3drobotics.com>
 *
 * @section DESCRIPTION
 *
 * Object to interpret information out of MAVLink messages and into
 * the Vehicle model
 */

#ifndef _ANALYZING_MAVLINK_MESSAGE_HANDLER_H
#define _ANALYZING_MAVLINK_MESSAGE_HANDLER_H

#include "mavlink_message_handler.h"
#include "analyze.h"

/// @brief Class to take data from decoded MAVLink messages, update
/// the vehicle model and call the analyzers
///
/// Add methods to this class to handle more MAVLink message types
class Analyzing_MAVLink_Message_Handler : public MAVLink_Message_Handler {

public:
    /// @brief construct an object to take MAVLink messages, update the vehicle model and call analyzers on the vehicle model
    /// @param analyze object responsible for vehicle analysys
    /// @param vehicle vehicle to analyze
    Analyzing_MAVLink_Message_Handler(Analyze *analyze,
                                      AnalyzerVehicle::Base *&vehicle);

private:

    virtual void handle_decoded_message_scaled_pressure(uint64_t T, const char *name, double press_abs, double temperature);

    virtual void handle_decoded_message(uint64_t T, mavlink_ahrs2_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_attitude_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_ekf_status_report_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_gps_raw_int_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_global_position_int_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_message_t &m UNUSED, mavlink_heartbeat_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_nav_controller_output_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_param_value_t &msg) override;    
    virtual void handle_decoded_message(uint64_t T, mavlink_power_status_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_scaled_pressure_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_scaled_pressure2_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_servo_output_raw_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_statustext_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_sys_status_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_system_time_t &msg) override;
    virtual void handle_decoded_message(uint64_t T, mavlink_vfr_hud_t &msg) override;

    /// @brief called when no more messages will be forthcoming
    void end_of_log(uint32_t packet_count, uint64_t bytes_dropped = 0) override;
    
    Analyze *_analyze;
    AnalyzerVehicle::Base *&_vehicle;

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
