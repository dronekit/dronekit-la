#include "LA_MsgHandler.h"

#include <inttypes.h>

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

AnalyzerVehicle::IMU* LA_MsgHandler::imu() {
    return _vehicle->imu(name());
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
    // ::fprintf(stderr, "type=%s T=%" PRIu64 "\n", name, time_us);

    if (_vehicle->T()) {
        // if log-when-disarmed is not set then you may end up with
        // large gaps in your dataflash data.  We *could* check for
        // whether we're armed or not before discarding messages, but depending on LOG_BITMASK and lost data we could end up discarding vast amounts of data.
        // uint64_t timestamp_max_delta = 100000000;
        uint64_t timestamp_max_delta = 1e010; // 1e10 ~= 167 minutes
        if (time_us < _vehicle->T()) {
            ::fprintf(stderr, "Time going backwards? (%" PRIu64 " < %" PRIu64 "); skipping packet\n", time_us, _vehicle->T());
            return false;
        }
        if (time_us - _vehicle->T() > timestamp_max_delta) { // 100 seconds
            ::fprintf(stderr, "Message timestamp bad (%" PRIu64 ") (%" PRIu64 " - %" PRIu64 " > %" PRIu64 "); skipping packet\n", time_us, time_us, _vehicle->T(), timestamp_max_delta);
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

LA_MsgHandler_AHR2::LA_MsgHandler_AHR2(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
    LA_MsgHandler(name, f, analyze, vehicle) {
    _analyze->add_data_source("ATTITUDE_ESTIMATE_AHR2", "AHR2.Roll");
    _analyze->add_data_source("ATTITUDE_ESTIMATE_AHR2", "AHR2.Pitch");
    _analyze->add_data_source("ATTITUDE_ESTIMATE_AHR2", "AHR2.Yaw");

    _analyze->add_data_source("POSITION_ESTIMATE_AHR2", "AHR2.Lat");
    _analyze->add_data_source("POSITION_ESTIMATE_AHR2", "AHR2.Lng");

    _analyze->add_data_source("ALTITUDE_ESTIMATE_AHR2", "AHR2.Alt");
}
void LA_MsgHandler_AHR2::xprocess(const uint8_t *msg) {
    int16_t Roll = require_field_int16_t(msg, "Roll");
    int16_t Pitch = require_field_int16_t(msg, "Pitch");
    float Yaw = require_field_float(msg, "Yaw");

    attitude_estimate()->set_roll(T(), Roll/(double)100.0f);
    attitude_estimate()->set_pitch(T(), Pitch/(double)100.0f);
    attitude_estimate()->set_yaw(T(), Yaw-180);

    int32_t Lat = require_field_int32_t(msg, "Lat");
    int32_t Lng = require_field_int32_t(msg, "Lng");
    float Alt = require_field_float(msg, "Alt");

    position_estimate()->set_lat(T(), Lat/(double)10000000.0f);
    position_estimate()->set_lon(T(), Lng/(double)10000000.0f);
    altitude_estimate()->set_alt(T(), Alt);

    double lat = Lat/(double)10000000.0f;
    double lng = Lng/(double)10000000.0f;
    if (canonical_for_position()) {
        _vehicle->set_lat(lat);
        _vehicle->set_lon(lng);
        _vehicle->set_altitude(Alt);
    }
    if (canonical_for_origin()) {
        if (_vehicle->is_armed()) {
            if (!_was_armed) {
                _was_armed = true;
                _vehicle->set_origin_lat(lat);
                _vehicle->set_origin_lon(lng);
                _vehicle->set_origin_altitude(Alt);
            }
        } else {
            _was_armed = false;
        }
    }
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
}

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


LA_MsgHandler_EKF1::LA_MsgHandler_EKF1(std::string name,
                                       const struct log_Format &f,
                                       Analyze *analyze,
                                       AnalyzerVehicle::Base *&vehicle) :
    LA_MsgHandler(name, f, analyze, vehicle) {
    _analyze->add_data_source("ATTITUDE_ESTIMATE_EKF1", "EKF1.Roll");
    _analyze->add_data_source("ATTITUDE_ESTIMATE_EKF1", "EKF1.Pitch");
    _analyze->add_data_source("ATTITUDE_ESTIMATE_EKF1", "EKF1.Yaw");

    _analyze->add_data_source("POSITION_ESTIMATE_EKF1", "EKF1.PN");
    _analyze->add_data_source("POSITION_ESTIMATE_EKF1", "EKF1.PE");

    _analyze->add_data_source("ALTITUDE_ESTIMATE_EKF1", "EKF1.PD");
}

void LA_MsgHandler_EKF1::xprocess(const uint8_t *msg) {
    int16_t Roll = require_field_int16_t(msg, "Roll");
    int16_t Pitch = require_field_int16_t(msg, "Pitch");
    float Yaw = require_field_float(msg, "Yaw");

    _vehicle->attitude_estimate("EKF1")->set_roll(T(), Roll/(double)100.0f);
    _vehicle->attitude_estimate("EKF1")->set_pitch(T(), Pitch/(double)100.0f);
    _vehicle->attitude_estimate("EKF1")->set_yaw(T(), Yaw-180);

    // these are all relative; need to work out an origin:
    if (_vehicle->origin_lat_T() != 0) {
        double posN = require_field_float(msg, "PN");
        double posE = require_field_float(msg, "PE");
        double origin_lat = _vehicle->origin_lat();
        double origin_lon = _vehicle->origin_lon();

        double lat = 0;
        double lon = 0;
        gps_offset(origin_lat, origin_lon, posE, posN, lat, lon);
        // ::fprintf(stderr, "%f+%f / %f+%f = %f / %f\n",
        //           origin_lat, posE, origin_lon, posN, lat, lon);

        _vehicle->position_estimate("EKF1")->set_lat(T(), lat);
        _vehicle->position_estimate("EKF1")->set_lon(T(), lon);
    }
    if (_vehicle->origin_altitude_T() != 0) {
        double posD = require_field_float(msg, "PD");
        double origin_alt = _vehicle->origin_altitude();
        _vehicle->altitude_estimate("EKF1")->set_alt(T(), origin_alt - posD);
    }

    // currently EKF1 is canonical for velocity:
    {
        _vehicle->vel().set_x(T(), require_field_float(msg, "VN"));
        _vehicle->vel().set_y(T(), require_field_float(msg, "VE"));
        _vehicle->vel().set_z(T(), require_field_float(msg, "VD"));
    }
    
}

#define ERROR_SUBSYSTEM_FAILSAFE_BATT       6
#define ERROR_SUBSYSTEM_CRASH	    	   12
#define ERROR_CODE_FAILSAFE_OCCURRED        1

LA_MsgHandler_ERR::LA_MsgHandler_ERR(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
    LA_MsgHandler(name, f, analyze, vehicle) {
    _analyze->add_data_source("BATTERY_FAILSAFE", "ERR.Subsys");
    _analyze->add_data_source("BATTERY_FAILSAFE", "ERR.ECode");
    _analyze->add_data_source("CRASH", "ERR.Subsys");
    _analyze->add_data_source("CRASH", "ERR.ECode");
}

void LA_MsgHandler_ERR::xprocess(const uint8_t *msg) {
    uint8_t subsys = require_field_uint8_t(msg, "Subsys");
    uint8_t ecode = require_field_uint8_t(msg, "ECode");
    switch(subsys) {
    case ERROR_SUBSYSTEM_FAILSAFE_BATT:
        _vehicle->set_battery_in_failsafe(ecode ? ERROR_CODE_FAILSAFE_OCCURRED : 0);
        break;
    case ERROR_SUBSYSTEM_CRASH:
        _vehicle->set_crashed(ecode);
        break;
    }
}

LA_MsgHandler_IMU::LA_MsgHandler_IMU(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
    LA_MsgHandler(name, f, analyze, vehicle) {

    // add gyroscope data sources
    _analyze->add_data_source(string_format("IMU_%s_GYR",name.c_str()),
                              string_format("%s.GyrX",name.c_str()));
    _analyze->add_data_source(string_format("IMU_%s_GYR",name.c_str()),
                              string_format("%s.GyrY",name.c_str()));
    _analyze->add_data_source(string_format("IMU_%s_GYR",name.c_str()),
                              string_format("%s.GyrZ",name.c_str()));
}

void LA_MsgHandler_IMU::xprocess(const uint8_t *msg) {
    Vector3f values;
    require_field(msg, "Gyr", values);

    imu()->set_gyr(T(), values);
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

void LA_MsgHandler_MAG::xprocess(const uint8_t *msg) {
    AnalyzerVehicle::Compass *compass = _vehicle->compass(_name);
    if (!field_value(msg, "Mag", compass->field())) {
        ::fprintf(stderr, "Failed to extract Mag fields from MAG message");
        abort();
    }
    compass->set_field_T(_vehicle->T());

    uint8_t healthy;
    if (field_value(msg, "Health", healthy)) {
        _vehicle->sensor_set_healthy(_name, healthy);
    }
}

void LA_MsgHandler_PM::xprocess(const uint8_t *msg)
{
    uint16_t nlon;
    if (field_value(msg, "NLon", nlon)) {
        // copter-style PM message
        _vehicle->autopilot_set_overruns(nlon);
        _vehicle->autopilot_set_loopcount(require_field_uint16_t(msg,"NLoop"));
        _vehicle->autopilot_set_slices_max(require_field_uint32_t(msg,"MaxT"));
    }
}


void LA_MsgHandler_VIBE::xprocess(const uint8_t *msg)
{
    if (!have_added_VIBE) {
        _analyze->add_data_source("IMU_VIBE_0", "VIBE.Clip0");
        _analyze->add_data_source("IMU_VIBE_1", "VIBE.Clip1");
        _analyze->add_data_source("IMU_VIBE_2", "VIBE.Clip2");
        have_added_VIBE = true;
    }

    for (uint8_t i=0; i<=2; i++) {
        // TODO: make sure we get the same IMU name as for ACC messages!
        const std::string imu_name = string_format("IMU_%d", i);
        AnalyzerVehicle::IMU *imu = _vehicle->imu(imu_name);

        const std::string field_name = string_format("Clip%d", i);
        const uint16_t count = require_field_uint16_t(msg, field_name.c_str());
        imu->set_acc_clip_count(count);
    }
}
