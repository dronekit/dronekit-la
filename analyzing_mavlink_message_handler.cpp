#include "analyzing_mavlink_message_handler.h"

void Analyzing_MAVLink_Message_Handler::handle_decoded_message(uint64_t T, mavlink_ahrs2_t &msg) {
    _vehicle->set_T(T);
}

void Analyzing_MAVLink_Message_Handler::end_of_log(uint32_t packet_count)
{
    _analyze->end_of_log(packet_count);
}

void Analyzing_MAVLink_Message_Handler::handle_decoded_message(uint64_t T, mavlink_attitude_t &msg) {
    _vehicle->set_T(T);

    _vehicle->set_roll(rad_to_deg(msg.roll));
    _vehicle->set_pitch(rad_to_deg(msg.pitch));
    _vehicle->set_yaw(rad_to_deg(msg.yaw)); // is this right?!

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

    _analyze->evaluate_all();
}

void Analyzing_MAVLink_Message_Handler::handle_decoded_message(uint64_t T, mavlink_vfr_hud_t &msg) {
    _vehicle->set_T(T);

    _vehicle->set_alt(msg.alt);

    _analyze->evaluate_all();
}

// something like this is in analyzing_dataflash_mavlink_message_handler
void Analyzing_MAVLink_Message_Handler::set_vehicle_copter()
{
    AnalyzerVehicle::Base *vehicle_old = _vehicle;
    AnalyzerVehicle::Copter *vehicle_new = new AnalyzerVehicle::Copter();
    vehicle_new->take_state(vehicle_old);
    _vehicle = vehicle_new;
    delete vehicle_old;
    _vehicle->set_vehicletype(AnalyzerVehicle::Base::vehicletype_t::copter);
}

void Analyzing_MAVLink_Message_Handler::handle_decoded_message(uint64_t T, mavlink_statustext_t &msg) {
    _vehicle->set_T(T);

    if (strstr(msg.text, "APM:Copter") || strstr(msg.text, "ArduCopter")) {
        set_vehicle_copter();
    }

    switch (_vehicle->vehicletype()) {
    case AnalyzerVehicle::Base::vehicletype_t::copter:
        if (strstr(msg.text, "Frame")) {
            ((AnalyzerVehicle::Copter*&)_vehicle)->set_frame(msg.text);
        }
        break;
    case AnalyzerVehicle::Base::vehicletype_t::invalid:
        break;
    }
}
