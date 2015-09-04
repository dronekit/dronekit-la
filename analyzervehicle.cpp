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

void Base::set_T(const uint64_t time_us)
{
    if (time_us < _T) {
        ::fprintf(stderr, "time going backwards\n");
        abort();
    }
    _T = time_us;
    // ::fprintf(stderr, "Set T to (%lu)\n", T);
}

bool Base::param(const char *name, float &ret)
{
    if (!param_seen(name)) {
        return false;
    }
    ret = param(name);
    return true;
}    

void Base::param_set(const char *name, const float value)
{
    // ::fprintf(stderr, "T=%lu %s=%f\n", T(), name, value);
    _param[name] = value;
    _param_modtime[name] = T();
}

void Base::take_state(Base *old)
{
    _param = old->_param;
    _param_modtime = old->_param_modtime;
}

bool Base::param_seen(const char *name) const
{
    const std::string x = std::string(name);
    // ::fprintf(stderr, "Looking for (%s)\n", name);
    std::map<const std::string, float>::const_iterator it = _param.find(name);
    if (it != _param.end()) {
        return true;
    }
    return false;
}

void Base::set_servo_output(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4, uint16_t ch5, uint16_t ch6, uint16_t ch7, uint16_t ch8)
{
    _servo_output[1] = (float)ch1;
    _servo_output[2] = (float)ch2;
    _servo_output[3] = (float)ch3;
    _servo_output[4] = (float)ch4;
    _servo_output[5] = (float)ch5;
    _servo_output[6] = (float)ch6;
    _servo_output[7] = (float)ch7;
    _servo_output[8] = (float)ch8;
}

void Base::set_servo_output(const uint8_t channel_number, const uint16_t value)
{
    _servo_output[channel_number] = (float)value;
}
