#include "analyzervehicle.h"

using namespace AnalyzerVehicle;

#define __STDC_FORMAT_MACROS 1 // for e.g. %PRIu64
#include "inttypes.h"

#include "analyzer_util.h"
#include "analyzervehicle_copter.h"
#include "analyzervehicle_plane.h"
#include "analyzervehicle_rover.h"

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
    case rover:
        vehicle_new = new AnalyzerVehicle::Rover();
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
        ::fprintf(stderr, "time going backwards (%" PRIu64 " < %" PRIu64 ") (delta=%" PRIi64 ")\n", time_us, _T, time_us - _T);
       return;
    }
    _T = time_us;
    // ::fprintf(stderr, "Set T to (%lu)\n", T);
}

bool Base::param_default(const char *name, float &ret) const
{
    auto it = _param_defaults.find(name);
    if (it != _param_defaults.end()) {
        ret = it->second;
        return true;
    }
    return false;
}

// will return value of param if seen, otherwise a default value
bool Base::param_with_defaults(const char *name, float &ret) const
{
    if (param_seen(name)) {
        ret = param(name);
        return true;
    }

    return param_default(name, ret);
}

uint64_t Base::param_T(const std::string name) const {
    auto it = _param_T.find(name);
    if (it != _param_T.end()) {
        return it->second;
    }
    ::fprintf(stderr, "param_T called for non-existant parameter (%s)\n", name.c_str());
    return 0;
}

// will return value of param if seen, otherwise a default value
float Base::require_param_with_defaults(const char *name) const
{
    if (param_seen(name)) {
        return param(name);
    }

    float ret;

    if (!param_default(name, ret)) {
        ::fprintf(stderr, "No %s parameter", name);
        abort();
    }

    return ret;
}

bool Base::param(const char *name, float &ret) const
{
    if (!param_seen(name)) {
        return false;
    }
    ret = param(name);
    return true;
}

bool Base::param(const std::string name, float &ret) const
{
    const char *xname = name.c_str();
    if (!param_seen(xname)) {
        return false;
    }
    ret = param(xname);
    return true;
}

float Base::param(const std::string name) const
{
    const std::string x = std::string(name);
    // ::fprintf(stderr, "Looking for (%s)\n", name);
    auto it = _param.find(name);
    if (it == _param.end()) {
        ::fprintf(stderr, "asked for unseen parameter");
        abort();
    }
    return it->second;
}

void Base::param_set(const char *name, const float value)
{
    // ::fprintf(stderr, "T=%lu %s=%f\n", T(), name, value);
    _param[name] = value;
    _param_T[name] = T();
}

void Base::take_state(Base *old)
{
  _param.insert(old->_param.begin(), old->_param.end());
  _param_T.insert(old->_param_T.begin(), old->_param_T.end());
  set_armed(old->is_armed());
}

bool Base::param_seen(const std::string name) const
{
    const std::string x = std::string(name);
    // ::fprintf(stderr, "Looking for (%s)\n", name);
    auto it = _param.find(name);
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

bool Base::relative_alt(double &relative) const
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

// if the gyro has ever clipped, returns that time
// otherwise returns 0
uint64_t AnalyzerVehicle::IMU::last_acc_clip_time() const
{
    if (_acc_clip_count == 0) {
        return 0;
    }
    return _acc_clip_count_T;
}

// set the number of times this acc has clipped
void AnalyzerVehicle::IMU::set_acc_clip_count(const uint16_t count)
{
    if (count > _acc_clip_count) {
        _acc_clip_count = count;
        _acc_clip_count_T = _T;
    } else if (count < _acc_clip_count) {
        // fprintf(stderr, "Weird: clip count going bacwards");
    }
}

// returns true if any clipping event happened in the last 0.1 seconds
bool AnalyzerVehicle::IMU::acc_is_clipping() const
{
    uint64_t last_clip_time = last_acc_clip_time();
    if (last_clip_time == 0) {
        return false;
    }
    if (T() - last_clip_time < 100000) { // FIXME: magic number
        return true;
    }

    return false;
}

void AnalyzerVehicle::IMU::set_gyr(const uint64_t T, const Vector3f gyr)
{
    // ::fprintf(stderr, "Using timestamp (%lu) for timestamp\n", T);
    _gyr_hist[_gyr_next].gyr = gyr;
    _gyr_hist[_gyr_next].T = T;
    _gyr_next++;
    if (_gyr_next >= _gyr_hist_max) {
        _gyr_next = 0;
    }
    _gyr_T = T; // FIXME: just take the most recent entry
    if (_gyr_seen < _gyr_hist_max) {
        _gyr_seen++;
    }
}

bool AnalyzerVehicle::IMU::gyr_avg(const uint16_t count, Vector3f &ret) const {
    uint16_t  offset = _gyr_next;
    uint32_t used = 0;
    ret = { };
    if (count > _gyr_hist_max) {
        ::fprintf(stderr, "insufficient history (%d > %d)", count, _gyr_hist_max);
        abort();
    }
    if (_gyr_seen < count) {
        return false;
    }
    while (used < count) {
        if (offset == 0) {
            offset = _gyr_hist_max-1;
        } else {
            offset--;
        }
        ret += _gyr_hist[offset].gyr;
        used++;
    }
    ret /= used;
    return true;
}
bool AnalyzerVehicle::IMU::gyr_avg(const uint64_t T, const uint64_t delta_T, Vector3f &ret) const {
    uint16_t  offset = _gyr_next;
    uint32_t used = 0;
    ret = { };

    if (delta_T > T) {
        return false;
    }
    uint64_t not_before = T - delta_T;

    while (true) {
        if (used >= _gyr_seen) {
            return false;
        }
        if (offset == 0) {
            offset = _gyr_hist_max-1;
        } else {
            offset--;
        }
        if (_gyr_hist[offset].T < not_before) {
            if (_gyr_hist[offset].T == 1000) {
                abort();
            }
            break;
        }
        ret += _gyr_hist[offset].gyr;
        used++;
    }
    ret /= used;
    return true;
}

bool Base::any_acc_clipping() const
{
    for (auto x = _imus.begin(); x != _imus.end(); x++) {
        if ((*x).second->acc_is_clipping()) {
            return true;
        }
    }
    return false;
}

double AnalyzerVehicle::Base::distance_from_origin()
{
    if (pos().is_zero_zero() || origin().is_zero_zero()) {
        return -1;
    }
    // ::fprintf(stderr, "distance: %f\n", pos().horizontal_distance_to(origin()));
    return pos().horizontal_distance_to(origin());
}

uint64_t AnalyzerVehicle::EKF::variance_T(std::string name) const {
    auto it = _variances_T.find(name);
    if (it != _variances_T.end()) {
        return it->second;
    }
    return 0;
}

void AnalyzerVehicle::AutoPilot::set_overruns(uint64_t T, uint16_t overruns)
{
    _overruns = overruns;
    _overruns_T = T;
}
void AnalyzerVehicle::AutoPilot::set_loopcount(uint64_t T, uint16_t loopcount)
{
    _loopcount = loopcount;
    _loopcount_T = T;
}
void AnalyzerVehicle::AutoPilot::set_slices_max(uint64_t T, uint16_t slices_max)
{
    _slices_max = slices_max;
    _slices_max_T = T;
}
void AnalyzerVehicle::AutoPilot::set_slices_min(uint64_t T, uint16_t slices_min)
{
    _slices_min = slices_min;
    _slices_min_T = T;
}
void AnalyzerVehicle::AutoPilot::set_slices_avg(uint64_t T, uint16_t slices_avg)
{
    _slices_avg = slices_avg;
    _slices_avg_T = T;
}
void AnalyzerVehicle::AutoPilot::set_slices_stddev(uint64_t T, uint16_t slices_stddev)
{
    _slices_stddev = slices_stddev;
    _slices_stddev_T = T;
}

void AnalyzerVehicle::AutoPilot::set_vcc(uint64_t T, double vcc)
{
    _vcc = vcc;
    _vcc_T = T;
}
