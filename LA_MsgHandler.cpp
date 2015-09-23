#include "LA_MsgHandler.h"

bool LA_MsgHandler::find_T(const uint8_t *msg, uint64_t &T)
{
    uint32_t time_ms;
    if (field_value(msg, "TimeUS", T)) {
    } else if (field_value(msg, "TimeMS", time_ms)) {
        // this is going to screw up on GPS messages...
        T = (uint64_t)(time_ms) * 1000;
    } else {
        // no timestamp...
        return false;
    }
    return true;
}

AnalyzerVehicle::AltitudeEstimate*
LA_MsgHandler::altitude_estimate() {
    return _vehicle->altitude_estimate(name());
}
AnalyzerVehicle::AttitudeEstimate*
LA_MsgHandler::attitude_estimate() {
    return _vehicle->attitude_estimate(name());
}
AnalyzerVehicle::PositionEstimate*
LA_MsgHandler::position_estimate() {
    return _vehicle->position_estimate(name());
}

bool LA_MsgHandler::process_set_T(const uint8_t *msg)
{
    uint64_t time_us;

    if (! find_T(msg, time_us)) {
        // no timestamp is OK!  Does not make the message invalid.
        return true;
    }

    // char name[1024];
    // string_for_labels(name, 1024);
    // ::fprintf(stderr, "type=%s T=%lu\n", name, time_us);

    if (_vehicle->T()) {
        uint64_t timestamp_max_delta = 100000000;
        if (time_us < _vehicle->T()) {
            ::fprintf(stderr, "Time going backwards? (%lu < %lu); skipping packet\n", time_us, _vehicle->T());
            return false;
        }
        if (time_us - _vehicle->T() > timestamp_max_delta) { // 100 seconds
            ::fprintf(stderr, "Message timestamp bad (%lu) (%lu - %lu > %lu); skipping packet\n", time_us, time_us, _vehicle->T(), timestamp_max_delta);
            return false;
        }
    }
    _vehicle->set_T(time_us);
    return true;
}

bool LA_MsgHandler_ACC::find_T(const uint8_t *msg, uint64_t &T) {
    uint32_t time_ms;
    if (field_value(msg, "TimeMS", time_ms)) {
        T = (uint64_t)(time_ms) * 1000;
        return true;
    }
    if (field_value(msg, "TimeUS", T)) {
        return true;
    }
    return false;
}

bool LA_MsgHandler_GPS::find_T(const uint8_t *msg, uint64_t &T) {
    if (field_value(msg, "TimeUS", T)) {
        return true;
    }

    uint32_t timestamp;
    require_field(msg, "T", timestamp);
    T = (uint64_t)timestamp * 1000;
    return true;
}
void LA_MsgHandler_GPS::xprocess(const uint8_t *msg) {
    int32_t Lat = require_field_int32_t(msg, "Lat");
    int32_t Lng = require_field_int32_t(msg, "Lng");
    int32_t Alt = require_field_int32_t(msg, "Alt");

    position_estimate()->set_lat(T(), Lat/10000000.0f);
    position_estimate()->set_lon(T(), Lng/10000000.0f);
    altitude_estimate()->set_alt(T(), Alt/100.0f);
}


bool LA_MsgHandler_GYR::find_T(const uint8_t *msg, uint64_t &T) {
    uint32_t time_ms;
    if (field_value(msg, "TimeMS", time_ms)) {
        T = (uint64_t)time_ms * 1000;
        return true;
    }
    if (field_value(msg, "TimeUS", T)) {
        return true;
    }
    return false;
}
