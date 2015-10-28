#include "analyzervehicle.h"

using namespace AnalyzerVehicle;

#include "analyzer_util.h"
#include "analyzervehicle_copter.h"
#include "analyzervehicle_plane.h"

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

void Base::switch_vehicletype(Base *&_vehicle, vehicletype_t newtype) {
    AnalyzerVehicle::Base *vehicle_new;
    switch (newtype) {
    case copter:
        vehicle_new = new AnalyzerVehicle::Copter();
        break;
    case plane:
        vehicle_new = new AnalyzerVehicle::Plane();
        break;
    default:
        ::fprintf(stderr, "unknown type");
        abort();
    }
    AnalyzerVehicle::Base *vehicle_old = _vehicle;
    vehicle_new->take_state(vehicle_old);
    _vehicle = vehicle_new;
    delete vehicle_old;
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

bool Base::param_default(const char *name, float &ret)
{
    if (_param_defaults.count(name)) {
        ret = _param_defaults[name];
        return true;
    }
    return false;
}

// will return value of param if see, otherwise a default value
bool Base::param_with_defaults(const char *name, float &ret)
{
    if (param_seen(name)) {
        ret = param(name);
        return true;
    }

    return param_default(name, ret);
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

bool Base::param_seen(const std::string name) const
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

bool Base::relative_alt(double &relative)
{
    if (origin_altitude_T() == 0) {
        return false;
    }
    if (alt().alt_modtime() == 0) {
        return false;
    }

    relative = alt().alt() - origin_altitude();
    return true;
}

void Base::set_servo_output(const uint8_t channel_number, const uint16_t value)
{
    _servo_output[channel_number] = (float)value;
}

// haversine formula:
// http://www.movable-type.co.uk/scripts/latlong.html
double haversine(double from_lat, double from_lon, double to_lat, double to_lon) {
    double lat_delta_radians = deg_to_rad(from_lat - to_lat);
    double lon_delta_radians = deg_to_rad(from_lon - to_lon);
    double sin_lat_delta_radians = sin(lat_delta_radians * 0.5);
    sin_lat_delta_radians *= sin_lat_delta_radians;
    double sin_lon_delta_radians = sin(lon_delta_radians * 0.5);
    sin_lon_delta_radians *= sin_lon_delta_radians;
    return 2.0 * asin(sqrt(sin_lat_delta_radians +
                           cos(deg_to_rad(from_lat)) * cos(deg_to_rad(to_lat))*sin_lon_delta_radians));
}

double AnalyzerVehicle::Position::horizontal_distance_to(AnalyzerVehicle::Position otherpos) const
{
    return (double)earthradius() * (haversine(lat(), lon(), otherpos.lat(), otherpos.lon()));
}

double AnalyzerVehicle::Base::distance_from_origin()
{
    if (pos().is_zero_zero() || origin().is_zero_zero()) {
        return -1;
    }
    // ::fprintf(stderr, "distance: %f\n", pos().horizontal_distance_to(origin()));
    return pos().horizontal_distance_to(origin());
}
