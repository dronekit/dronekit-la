#include "analyzervehicle.h"

using namespace AnalyzerVehicle;

void Base::handle_decoded_message(uint64_t T, mavlink_statustext_t &msg)
{
    history_statustext.packet(msg);
}

void Base::handle_decoded_message(uint64_t T, mavlink_heartbeat_t &msg)
{
    history_heartbeat.packet(msg);
    _armed = msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
}

void Base::handle_decoded_message(uint64_t T, mavlink_servo_output_raw_t &msg)
{
    history_servo_output_raw.packet(msg);
}

char *
xstrdup(const char *x)
{
    char *ret = strdup(x);
    if (ret == NULL) {
        fprintf(stderr, "Failed to strdup: %s", strerror(errno));
        abort();
    }
    return ret;
}

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



void Base::handle_decoded_message(uint64_t T, mavlink_param_value_t &msg) {
    // FIXME: getting the same parameter multiple times leaks memory
    char *str = xcalloc(17); // FIXME constant
    memcpy(str, msg.param_id, 16);
    params[str] = msg.param_value;
}

void Base::handle_decoded_message(uint64_t T, mavlink_attitude_t &msg)
{
    att().set_roll(T, msg.roll);
    att().set_pitch(T, msg.pitch);
    att().set_yaw(T, msg.yaw);
}

bool Base::seen_parameter(const char *name)
{
    // ::printf("Looking for (%s)\n", name);
    std::map<std::string, float>::iterator it = params.find(std::string(name));
   if (it != params.end()) {
       return true;
   }
   return false;
}
