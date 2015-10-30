#include "analyzing_mavlink_message_handler.h"

void Analyzing_MAVLink_Message_Handler::end_of_log(uint32_t packet_count, uint64_t bytes_dropped UNUSED)
{
    _analyze->end_of_log(packet_count);
}


void Analyzing_MAVLink_Message_Handler::handle_decoded_message(uint64_t T, mavlink_ahrs2_t &msg) {

    double lat = msg.lat/(double)10000000.0f;
    double lng = msg.lng/(double)10000000.0f;
    double alt = msg.altitude;

    _vehicle->position_estimate("AHRS2")->set_lat(T, lat);
    _vehicle->position_estimate("AHRS2")->set_lon(T, lng);
    _vehicle->altitude_estimate("AHRS2")->set_alt(T, alt);

    _vehicle->attitude_estimate("AHRS2")->set_roll(T, rad_to_deg(msg.roll));
    _vehicle->attitude_estimate("AHRS2")->set_pitch(T, rad_to_deg(msg.pitch));
    _vehicle->attitude_estimate("AHRS2")->set_yaw(T, rad_to_deg(msg.yaw));


    // we fake up the vehicle origin by setting it whenever the
    // vehicle moves from disarmed to armed
    bool is_armed = _vehicle->is_armed();
    if (is_armed) {
        if (!set_origin_was_armed) {
            _vehicle->set_origin_lat(lat);
            _vehicle->set_origin_lon(lng);
            _vehicle->set_origin_altitude(alt);
            set_origin_was_armed = true;
        }
    } else {
        _vehicle->set_origin_lat(0);
        _vehicle->set_origin_lon(0);
        _vehicle->set_origin_altitude(0);
        set_origin_was_armed = false;
    }

    _analyze->evaluate_all();
}

void Analyzing_MAVLink_Message_Handler::handle_decoded_message(uint64_t T, mavlink_attitude_t &msg) {
    _vehicle->set_T(T);

    _vehicle->attitude_estimate("ATTITUDE")->set_roll(T, rad_to_deg(msg.roll));
    _vehicle->attitude_estimate("ATTITUDE")->set_pitch(T, rad_to_deg(msg.pitch));
    _vehicle->attitude_estimate("ATTITUDE")->set_yaw(T, rad_to_deg(msg.yaw));

    _vehicle->set_roll(rad_to_deg(msg.roll));
    _vehicle->set_pitch(rad_to_deg(msg.pitch));
    _vehicle->set_yaw(rad_to_deg(msg.yaw));

    _analyze->evaluate_all();
}

void Analyzing_MAVLink_Message_Handler::handle_decoded_message(uint64_t T, mavlink_global_position_int_t &msg) {
    _vehicle->position_estimate("GLOBAL_POSITION_INT")->set_lat(T, msg.lat/(double)10000000.0f);
    _vehicle->position_estimate("GLOBAL_POSITION_INT")->set_lon(T, msg.lon/(double)10000000.0f);
    _vehicle->altitude_estimate("GLOBAL_POSITION_INT")->set_alt(T, msg.alt/(double)1000.0f);

    // GLOBAL_POSITION_INT is the "offical" position of the vehicle:
    double lat = msg.lat/(double)10000000.0f;
    double lon = msg.lon/(double)10000000.0f;
    _vehicle->set_T(T);
    _vehicle->set_lat(lat);
    _vehicle->set_lon(lon);
    // and I'm assuming for ALT as well:
    double alt = msg.alt/(double)1000.0f;
    _vehicle->set_altitude(alt);
    if (_vehicle->origin_altitude_T() == 0) {
        _vehicle->set_origin_altitude(alt);
    }

    // ... and for velocity
    _vehicle->vel().set_x(T, msg.vx/100.0f);
    _vehicle->vel().set_y(T, msg.vy/100.0f);
    _vehicle->vel().set_z(T, msg.vz/100.0f);

    _analyze->evaluate_all();
}

void Analyzing_MAVLink_Message_Handler::handle_decoded_message(uint64_t T, mavlink_gps_raw_int_t &msg) {
    _vehicle->position_estimate("GPS_RAW_INT")->set_lat(T, msg.lat/(double)10000000.0f);
    _vehicle->position_estimate("GPS_RAW_INT")->set_lon(T, msg.lon/(double)10000000.0f);
    _vehicle->altitude_estimate("GPS_RAW_INT")->set_alt(T, msg.alt/(double)1000.0f);

    _vehicle->gpsinfo("GPS_RAW_INT")->set_satellites(msg.satellites_visible);
    _vehicle->gpsinfo("GPS_RAW_INT")->set_hdop(msg.eph/(double)100.0f);
    _vehicle->gpsinfo("GPS_RAW_INT")->set_fix_type(msg.fix_type);

    _analyze->evaluate_all();
}

void Analyzing_MAVLink_Message_Handler::handle_decoded_message(uint64_t T, mavlink_heartbeat_t &msg) {
    _vehicle->set_T(T);

    if (msg.autopilot == 8) {
        // drop any invalid-autopilot message; this could possibly be moved into
        // all callees
        return;
    }

    _vehicle->set_armed(msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);

    _analyze->evaluate_all();
}

void Analyzing_MAVLink_Message_Handler::handle_decoded_message(uint64_t T, mavlink_nav_controller_output_t &msg) {
    _vehicle->set_T(T);

    _vehicle->set_desroll(msg.nav_roll);
    _vehicle->set_despitch(msg.nav_pitch);
    _vehicle->set_desyaw(msg.nav_bearing);

    _analyze->evaluate_all();
}

void Analyzing_MAVLink_Message_Handler::handle_decoded_message(uint64_t T, mavlink_param_value_t &msg) {
    _vehicle->set_T(T);

    char buf[17] = { };
    memcpy(buf, msg.param_id, 16);
    _vehicle->param_set(buf, msg.param_value);

    _analyze->evaluate_all();
}

void Analyzing_MAVLink_Message_Handler::handle_decoded_message_scaled_pressure(uint64_t T, const char *name, double press_abs, double temperature) {
    _vehicle->set_T(T);

    float gnd_abs_press;
    float gnd_temp;
    if (_vehicle->param("GND_ABS_PRESS", gnd_abs_press) &&
        _vehicle->param("GND_TEMP", gnd_temp) &&
        _vehicle->origin_altitude_T()) {

        double relalt = altitude_from_pressure_delta(
            gnd_abs_press/(double)100.0f,
            gnd_temp,
            press_abs,
            temperature);
        
        double alt = _vehicle->origin_altitude() + relalt;
        // ::fprintf(stderr, "SCALED_PRESSURE alt=%f\n", alt);
        _vehicle->altitude_estimate(name)->set_alt(T, alt);
    }

    _analyze->evaluate_all();
}

void Analyzing_MAVLink_Message_Handler::handle_decoded_message(uint64_t T, mavlink_scaled_pressure_t &msg) {
    handle_decoded_message_scaled_pressure(T, "SCALED_PRESSURE",
                                           msg.press_abs,
                                           msg.temperature);
}

void Analyzing_MAVLink_Message_Handler::handle_decoded_message(uint64_t T, mavlink_scaled_pressure2_t &msg) {
    handle_decoded_message_scaled_pressure(T, "SCALED_PRESSURE2",
                                           msg.press_abs,
                                           msg.temperature);
}

void Analyzing_MAVLink_Message_Handler::handle_decoded_message(uint64_t T, mavlink_servo_output_raw_t &msg) {
    _vehicle->set_T(T);

    _vehicle->set_servo_output(msg.servo1_raw,
                              msg.servo2_raw,
                              msg.servo3_raw,
                              msg.servo4_raw,
                              msg.servo5_raw,
                              msg.servo6_raw,
                              msg.servo7_raw,
                              msg.servo8_raw);

    _analyze->evaluate_all();
}

void Analyzing_MAVLink_Message_Handler::handle_decoded_message(uint64_t T, mavlink_ekf_status_report_t &msg) {
    _vehicle->set_T(T);

    _vehicle->ekf_set_variance("velocity", msg.velocity_variance);
    _vehicle->ekf_set_variance("pos_horiz", msg.pos_horiz_variance);
    _vehicle->ekf_set_variance("pos_vert", msg.pos_vert_variance);
    _vehicle->ekf_set_variance("compass", msg.compass_variance);
    _vehicle->ekf_set_variance("terrain_alt", msg.terrain_alt_variance);

    _vehicle->ekf_set_flags(msg.flags);

    _analyze->evaluate_all();
}

void Analyzing_MAVLink_Message_Handler::handle_decoded_message(uint64_t T, mavlink_sys_status_t &msg) {
    _vehicle->set_T(T);

    _vehicle->set_battery_remaining(msg.battery_remaining);

    for (std::map<const std::string, const uint64_t>::const_iterator it = _sensor_masks.begin();
         it != _sensor_masks.end();
         it++) {
        std::string name = (*it).first;
        uint64_t mask = (*it).second;
        // ::fprintf(stderr, "sensor: %s\n", name.c_str());
        // _vehicle->sensor_present(name, msg.onboard_control_sensors_present & mask);
        // _vehicle->sensor_enabled(name, msg.onboard_control_sensors_enabled & mask);
        // _vehicle->sensor_healthy(name, msg.onboard_control_sensors_health & mask);
        if (msg.onboard_control_sensors_present & mask &&
            msg.onboard_control_sensors_enabled & mask) {
            _vehicle->sensor_set_healthy(name, msg.onboard_control_sensors_health & mask);
        }
    }
    
    _analyze->evaluate_all();
}

void Analyzing_MAVLink_Message_Handler::handle_decoded_message(uint64_t T, mavlink_system_time_t &msg) {
    _vehicle->set_T(T);

    _vehicle->set_time_since_boot(msg.time_boot_ms * 1000);
    // _analyze->evaluate_all();
}

void Analyzing_MAVLink_Message_Handler::handle_decoded_message(uint64_t T, mavlink_vfr_hud_t &msg UNUSED) {
    _vehicle->set_T(T);

    // vfr_hud is a relative altitude, set_alt sets absolute altitude
//    _vehicle->set_alt(msg.alt);

    // _analyze->evaluate_all();
}

void Analyzing_MAVLink_Message_Handler::handle_decoded_message(uint64_t T, mavlink_statustext_t &msg) {
    _vehicle->set_T(T);

    if (!_vehicle->vehicletype_is_forced()) {
        AnalyzerVehicle::Base::vehicletype_t newtype = AnalyzerVehicle::Base::vehicletype_t::invalid;
        if (strstr(msg.text, "APM:Copter") || strstr(msg.text, "ArduCopter")) {
            newtype = AnalyzerVehicle::Base::vehicletype_t::copter;
        } else if (strstr(msg.text, "ArduPlane")) {
            newtype = AnalyzerVehicle::Base::vehicletype_t::plane;
        }
        if (newtype != AnalyzerVehicle::Base::vehicletype_t::invalid) {
            AnalyzerVehicle::Base::switch_vehicletype(_vehicle, newtype);
        }

        switch (_vehicle->vehicletype()) {
        case AnalyzerVehicle::Base::vehicletype_t::copter:
            if (strstr(msg.text, "Frame")) {
                ((AnalyzerVehicle::Copter*&)_vehicle)->set_frame(msg.text);
            }
            break;
        case AnalyzerVehicle::Base::vehicletype_t::plane:
            break;
        case AnalyzerVehicle::Base::vehicletype_t::invalid:
            break;
        }
    }
}
