#include "analyzervehicle.h"

using namespace AnalyzerVehicle;

#include "analyzer_util.h"

char *
xcalloc(size_t size)
{
    char *ret = (char*)calloc(1, size);
    if (ret == NULL) {
        fprintf(stderr, "Failed to calloc: %s", strerror(errno));
        abort();
    }
    return ret;
}

void Base::handle_decoded_message(uint64_t T, mavlink_attitude_t &msg)
{
    att().set_roll(T, rad_to_deg(msg.roll));
    att().set_pitch(T, rad_to_deg(msg.pitch));
    att().set_yaw(T, rad_to_deg(msg.yaw));
}
void Base::handle_decoded_message(uint64_t T, mavlink_heartbeat_t &msg)
{
    history_heartbeat.packet(msg);
    _armed = msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
}
void Base::handle_decoded_message(uint64_t T, mavlink_param_value_t &msg) {
    // FIXME: getting the same parameter multiple times leaks memory
    char *str = xcalloc(17); // FIXME constant
    memcpy(str, msg.param_id, 16);
    _param[str] = msg.param_value;
    _param_modtime[str] = T;
}
void Base::handle_decoded_message(uint64_t T, mavlink_nav_controller_output_t &msg)
{ 
    history_nav_controller_output.packet(msg);
    nav().set_desroll(T, msg.nav_roll);
    nav().set_despitch(T, msg.nav_pitch);
    nav().set_desyaw(T, msg.nav_bearing);
}
void Base::handle_decoded_message(uint64_t T, mavlink_servo_output_raw_t &msg)
{
    history_servo_output_raw.packet(msg);
}
void Base::handle_decoded_message(uint64_t T, mavlink_statustext_t &msg)
{
    history_statustext.packet(msg);
}

void Base::handle_decoded_message(uint64_t T, mavlink_vfr_hud_t &msg)
{
    pos().set_alt(T, msg.alt);
}


bool Base::param_seen(const char *name) const
{
    const std::string x = std::string(name);
    std::map<const std::string, float>::const_iterator it = _param.find(x);
    if (it != _param.end()) {
        return true;
    }
    return false;
}
