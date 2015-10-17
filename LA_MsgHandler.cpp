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

AnalyzerVehicle::GPSInfo*
LA_MsgHandler::gpsinfo() {
    return _vehicle->gpsinfo(name());
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
    _vehicle->set_time_since_boot(time_us);
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

LA_MsgHandler_ATT::LA_MsgHandler_ATT(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
    LA_MsgHandler(name, f, analyze, vehicle) {
    _analyze->add_data_source("ATTITUDE", "ATT.Roll");
    _analyze->add_data_source("ATTITUDE", "ATT.Pitch");
    _analyze->add_data_source("ATTITUDE", "ATT.Yaw");
    _analyze->add_data_source("DESATTITUDE", "ATT.DesRoll");
    _analyze->add_data_source("DESATTITUDE", "ATT.DesPitch");
    _analyze->add_data_source("DESATTITUDE", "ATT.DesYaw");

    _analyze->add_data_source("ATTITUDE_ESTIMATE_ATTITUDE", "ATT.Roll");
    _analyze->add_data_source("ATTITUDE_ESTIMATE_ATTITUDE", "ATT.Pitch");
    _analyze->add_data_source("ATTITUDE_ESTIMATE_ATTITUDE", "ATT.Yaw");
};

void LA_MsgHandler_ATT::xprocess(const uint8_t *msg) {
    int16_t DesRoll = require_field_int16_t(msg, "DesRoll");
    int16_t Roll = require_field_int16_t(msg, "Roll");
    int16_t DesPitch = require_field_int16_t(msg, "DesPitch");
    int16_t Pitch = require_field_int16_t(msg, "Pitch");
    uint16_t DesYaw = require_field_uint16_t(msg, "DesYaw");
    uint16_t Yaw = require_field_uint16_t(msg, "Yaw");

    // _vehicle->set_roll(rad_to_deg(Roll/100.0f));
    // _vehicle->set_pitch(rad_to_deg(Pitch/100.0f));
    _vehicle->set_roll(Roll/(double)100.0f);
    _vehicle->set_pitch(Pitch/(double)100.0f);
    _vehicle->set_yaw(Yaw);

    _vehicle->set_desroll((float)DesRoll/(double)100.0f);
    _vehicle->set_despitch((float)DesPitch/(double)100.0f);
    _vehicle->set_desyaw(DesYaw);

    _vehicle->attitude_estimate("ATT")->set_roll(T(), Roll/(double)100.0f);
    _vehicle->attitude_estimate("ATT")->set_pitch(T(), Pitch/(double)100.0f);
    _vehicle->attitude_estimate("ATT")->set_yaw(T(), Yaw);
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

    position_estimate()->set_lat(T(), Lat/(double)10000000.0f);
    position_estimate()->set_lon(T(), Lng/(double)10000000.0f);
    altitude_estimate()->set_alt(T(), Alt/(double)100.0f);

    uint8_t nsats;
    if (field_value(msg, "NSats", nsats) ||
        field_value(msg, "NSvv", nsats)) {
        // we have a value!
    } else {
        ::fprintf(stderr, "Unable to extract number of satellites visible from GPS message");
        abort();
    }
    gpsinfo()->set_satellites(nsats);
    gpsinfo()->set_hdop(require_field_int16_t(msg, "HDop")/(double)100.0f);
    gpsinfo()->set_fix_type(require_field_uint8_t(msg, "Status"));
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

void LA_MsgHandler_MSG::xprocess(const uint8_t *msg) {
    char msg_message[160];
    require_field(msg, "Message", msg_message, sizeof(msg_message));

    if (!_vehicle->vehicletype_is_forced()) {
        AnalyzerVehicle::Base::vehicletype_t newtype = AnalyzerVehicle::Base::vehicletype_t::invalid;
        if (strstr(msg_message, "APM:Copter") || strstr(msg_message, "ArduCopter")) {
            newtype = AnalyzerVehicle::Base::vehicletype_t::copter;
        } else if (strstr(msg_message, "ArduPlane")) {
            newtype = AnalyzerVehicle::Base::vehicletype_t::plane;
        }
        if (newtype != AnalyzerVehicle::Base::vehicletype_t::invalid) {
            AnalyzerVehicle::Base::switch_vehicletype(_vehicle, newtype);
        }

        switch(_vehicle->vehicletype()) {
        case AnalyzerVehicle::Base::vehicletype_t::copter:
            if (strstr(msg_message, "Frame")) {
                ((AnalyzerVehicle::Copter*&)_vehicle)->set_frame(msg_message);
            }
            break;
        case AnalyzerVehicle::Base::vehicletype_t::plane:
            break;
        case AnalyzerVehicle::Base::vehicletype_t::invalid:
            ::fprintf(stderr, "unhandled message (%s)\n", msg_message);
            // abort();
        }
    }
}

void LA_MsgHandler_STAT::xprocess(const uint8_t *msg)
{
    if (!have_added_STAT) {
        _analyze->add_data_source("ARMING", "STAT.Armed");
        have_added_STAT = true;
    }
    bool armed;
    if (field_value(msg, "Armed", armed)) {
        _vehicle->set_armed(armed);
    }
}
