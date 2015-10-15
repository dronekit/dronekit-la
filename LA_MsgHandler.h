#ifndef AP_LA_MSGHANDLER_H
#define AP_LA_MSGHANDLER_H

#include "MsgHandler.h"

#include "analyze.h"
#include "analyzervehicle.h"
#include "analyzervehicle_copter.h"
#include "analyzervehicle_plane.h"

#include <string.h>

class LA_MsgHandler : public MsgHandler {
public:
    LA_MsgHandler(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        MsgHandler(f),
        _name(name),
        _analyze(analyze),
        _vehicle(vehicle)
        { };

    virtual bool find_T(const uint8_t *msg, uint64_t &T);
    bool process_set_T(const uint8_t *msg);
    uint64_t T() const { return _vehicle->T(); } // only valid after process_set_T

    virtual void xprocess(const uint8_t *msg) = 0;

    virtual void process(const uint8_t *msg) {
        if (!process_set_T(msg)) {
            return;
        }
        xprocess(msg);
        _analyze->evaluate_all();
    }

    AnalyzerVehicle::AltitudeEstimate* altitude_estimate();
    AnalyzerVehicle::AttitudeEstimate* attitude_estimate();
    AnalyzerVehicle::PositionEstimate* position_estimate();
    AnalyzerVehicle::GPSInfo* gpsinfo();
    const std::string name() const { return _name; }

protected:
    std::string _name;
    Analyze *_analyze;
    AnalyzerVehicle::Base *&_vehicle;
};

// this is just here ATM because older messages have both TimeMS and
// TimeUS as 32-bit quantities.  I have logs where the TimeUS wraps!
class LA_MsgHandler_ACC : public LA_MsgHandler {
public:
    LA_MsgHandler_ACC(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(name, f, analyze, vehicle) {
    };

    bool find_T(const uint8_t *msg, uint64_t &T);
};

class LA_MsgHandler_AHR2 : public LA_MsgHandler {
public:
    LA_MsgHandler_AHR2(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(name, f, analyze, vehicle) {
        _analyze->add_data_source("ATTITUDE_ESTIMATE_AHR2", "AHR2.Roll");
        _analyze->add_data_source("ATTITUDE_ESTIMATE_AHR2", "AHR2.Pitch");
        _analyze->add_data_source("ATTITUDE_ESTIMATE_AHR2", "AHR2.Yaw");

        _analyze->add_data_source("POSITION_ESTIMATE_AHR2", "AHR2.Lat");
        _analyze->add_data_source("POSITION_ESTIMATE_AHR2", "AHR2.Lng");

        _analyze->add_data_source("ALTITUDE_ESTIMATE_AHR2", "AHR2.Alt");
    };
    void xprocess(const uint8_t *msg) override {
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
            if (_vehicle->origin_lat_T() == 0) {
                _vehicle->set_origin_lat(lat);
                _vehicle->set_origin_lon(lng);
                _vehicle->set_origin_altitude(Alt);
            }
        }
    }

    bool canonical_for_position() { return _canonical_for_position; };
    void set_canonical_for_position(bool value) { _canonical_for_position = value; }
    bool canonical_for_origin() { return _canonical_for_origin; };
    void set_canonical_for_origin(bool value) { _canonical_for_origin = value; }

private:
    bool _canonical_for_position = true;
    bool _canonical_for_origin = true;
};

class LA_MsgHandler_ATT : public LA_MsgHandler {
public:
    LA_MsgHandler_ATT(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle);
    void xprocess(const uint8_t *msg) override;
};

class LA_MsgHandler_BARO : public LA_MsgHandler {
public:
    LA_MsgHandler_BARO(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(name, f, analyze, vehicle) {
        _analyze->add_data_source("ALTITUDE_ESTIMATE_BARO", "BARO.Alt");
    };
    void xprocess(const uint8_t *msg) override {
        float Alt = require_field_float(msg, "Alt");

        if (_vehicle->origin_altitude_T() != 0) {
            double origin_alt = _vehicle->origin_altitude();
            _vehicle->altitude_estimate("BARO")->set_alt(T(), origin_alt + Alt);
        }
    }
};


class LA_MsgHandler_EKF1 : public LA_MsgHandler {
public:
    LA_MsgHandler_EKF1(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(name, f, analyze, vehicle) {
        _analyze->add_data_source("ATTITUDE_ESTIMATE_EKF1", "EKF1.Roll");
        _analyze->add_data_source("ATTITUDE_ESTIMATE_EKF1", "EKF1.Pitch");
        _analyze->add_data_source("ATTITUDE_ESTIMATE_EKF1", "EKF1.Yaw");

        _analyze->add_data_source("POSITION_ESTIMATE_EKF1", "EKF1.PN");
        _analyze->add_data_source("POSITION_ESTIMATE_EKF1", "EKF1.PE");

        _analyze->add_data_source("ALTITUDE_ESTIMATE_EKF1", "EKF1.PD");
    };
    void xprocess(const uint8_t *msg) override {
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
    }
};

class LA_MsgHandler_EKF4 : public LA_MsgHandler {
public:
    LA_MsgHandler_EKF4(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(name, f, analyze, vehicle) {
        _analyze->add_data_source("EKF_FLAGS", "EKF4.SS");
        _analyze->add_data_source("EKF_VARIANCES_velocity_variance", "EKF4.SV");
        _analyze->add_data_source("EKF_VARIANCES_pos_horiz_variance", "EKF4.SP");
        _analyze->add_data_source("EKF_VARIANCES_pos_vert_variance", "EKF4.SH");
        _analyze->add_data_source("EKF_VARIANCES_compass_variance", "EKF4.SMX");
        _analyze->add_data_source("EKF_VARIANCES_compass_variance", "EKF4.SMY");
        _analyze->add_data_source("EKF_VARIANCES_compass_variance", "EKF4.SMZ");
        _analyze->add_data_source("EKF_VARIANCES_terrain_alt_variance", "EKF4.SVT");
    };
    void xprocess(const uint8_t *msg) override {
        _vehicle->ekf_set_variance("velocity", require_field_uint16_t(msg, "SV") / (double)100.0f);
        _vehicle->ekf_set_variance("pos_horiz", require_field_uint16_t(msg, "SP") / (double)100.0f);
        _vehicle->ekf_set_variance("pos_vert", require_field_uint16_t(msg, "SH") / (double)100.0f);
        _vehicle->ekf_set_variance("terrain_alt", require_field_uint16_t(msg, "SVT") / (double)100.0f);

        double m[3];
        m[0] = require_field_uint16_t(msg, "SMX") / (double)100.0f;
        m[1] = require_field_uint16_t(msg, "SMY") / (double)100.0f;
        m[2] = require_field_uint16_t(msg, "SMZ") / (double)100.0f;

        _vehicle->ekf_set_variance("compass", vec_len(m));

        // SS came in somewhere between Copter 3.2.1 and 3.3rc9
        uint16_t ss;
        if (field_value(msg, "SS", ss)) {
            _vehicle->ekf_set_flags(ss);
        }
    }
};

#define ERROR_SUBSYSTEM_FAILSAFE_BATT       6
#define ERROR_CODE_FAILSAFE_OCCURRED        1

class LA_MsgHandler_ERR : public LA_MsgHandler {
public:
    LA_MsgHandler_ERR(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(name, f, analyze, vehicle) {
        _analyze->add_data_source("BATTERY_FAILSAFE", "ERR.Subsys");
        _analyze->add_data_source("BATTERY_FAILSAFE", "ERR.ECode");
    };

    void xprocess(const uint8_t *msg) override {
        uint8_t subsys = require_field_uint8_t(msg, "Subsys");
        uint8_t ecode = require_field_uint8_t(msg, "ECode");
        if (subsys == ERROR_SUBSYSTEM_FAILSAFE_BATT) {
            _vehicle->set_battery_in_failsafe(ecode ? ERROR_CODE_FAILSAFE_OCCURRED : 0);
        }
    }
};

class LA_MsgHandler_EV : public LA_MsgHandler {
public:
    LA_MsgHandler_EV(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(name, f, analyze, vehicle) {
        _analyze->add_data_source("ARMING", "EV.Id");
    };

    void xprocess(const uint8_t *msg) override {
        uint8_t Id = require_field_uint8_t(msg, "Id");
        switch(Id) {
        case 10:
            _vehicle->set_armed(true);
            break;
        case 11:
            _vehicle->set_armed(false);
            break;
        }
    }
};

class LA_MsgHandler_GPS : public LA_MsgHandler {
public:
    bool find_T(const uint8_t *msg, uint64_t &T);

    LA_MsgHandler_GPS(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(name, f, analyze, vehicle) {
        _analyze->add_data_source(string_format("POSITION_ESTIMATE_%s",name.c_str()), string_format("%s.Lat",name.c_str()));
        _analyze->add_data_source(string_format("POSITION_ESTIMATE_%s",name.c_str()), string_format("%s.Lng",name.c_str()));
        _analyze->add_data_source(string_format("ALTITUDE_ESTIMATE_%s",name.c_str()), string_format("%s.Alt",name.c_str()));

        _analyze->add_data_source(string_format("GPSINFO_%s",name.c_str()),
            string_format("%s.HDop",name.c_str()));
        // FIXME: need to take from correct source here!  Move to xprocess?
        _analyze->add_data_source(string_format("GPSINFO_%s",name.c_str()),
            string_format("%s.NSats",name.c_str()));
    };
    void xprocess(const uint8_t *msg) override;
};

// this is just here ATM because older messages have both TimeMS and
// TimeUS as 32-bit quantities.  I have logs where the TimeUS wraps!
class LA_MsgHandler_GYR : public LA_MsgHandler {
public:
    LA_MsgHandler_GYR(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(name, f, analyze, vehicle) {
    };

    bool find_T(const uint8_t *msg, uint64_t &T);
};

class LA_MsgHandler_MSG : public LA_MsgHandler {
public:
    LA_MsgHandler_MSG(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(name, f, analyze, vehicle) {
        _analyze->add_data_source("VEHICLE_DEFINITION", "MSG.Message");
    };

    void xprocess(const uint8_t *msg) override;
};

class LA_MsgHandler_ORGN : public LA_MsgHandler {
public:
    LA_MsgHandler_ORGN(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(name, f, analyze, vehicle) {
        _analyze->add_data_source("ORIGIN", "ORGN.Lat");
        _analyze->add_data_source("ORIGIN", "ORGN.Lng");
        _analyze->add_data_source("ORIGIN", "ORGN.Alt");
    };
    void xprocess(const uint8_t *msg) override {
        int32_t Lat = require_field_int32_t(msg, "Lat");
        int32_t Lng = require_field_int32_t(msg, "Lng");
        float Alt = require_field_float(msg, "Alt");

        double lat = Lat/(double)10000000.0f;
        double lng = Lng/(double)10000000.0f;
        double alt = Alt/(double)100.0f;

        _vehicle->set_origin_lat(lat);
        _vehicle->set_origin_lon(lng);
        _vehicle->set_origin_altitude(alt);
    }
};

class LA_MsgHandler_PARM : public LA_MsgHandler {
public:
    LA_MsgHandler_PARM(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(name, f, analyze, vehicle) {
        _analyze->add_data_source("PARAM", "PARM.Name");
        _analyze->add_data_source("PARAM", "PARM.Value");
    };

    void xprocess(const uint8_t *msg) override {
        const uint8_t namelen = 255;
        char name[namelen];

        memset(name, 0, namelen);
        require_field(msg, "Name", name, namelen);
        float value = require_field_float(msg, "Value");

        _vehicle->param_set(name, value);
    }

};

class LA_MsgHandler_POS : public LA_MsgHandler {
public:
    LA_MsgHandler_POS(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(name, f, analyze, vehicle) {
        _analyze->add_data_source("POSITION_ESTIMATE_POS", "POS.Lat");
        _analyze->add_data_source("POSITION_ESTIMATE_POS", "POS.Lng");
    };
    void xprocess(const uint8_t *msg) override {
        int32_t Lat = require_field_int32_t(msg, "Lat");
        int32_t Lng = require_field_int32_t(msg, "Lng");
        float Alt = require_field_float(msg, "Alt");

        _vehicle->position_estimate("POS")->set_lat(T(), Lat/(double)10000000.0f);
        _vehicle->position_estimate("POS")->set_lon(T(), Lng/(double)10000000.0f);
        _vehicle->altitude_estimate("POS")->set_alt(T(), Alt);

        double lat = Lat/(double)10000000.0f;
        double lng = Lng/(double)10000000.0f;

        _vehicle->set_lat(lat);
        _vehicle->set_lon(lng);
        _vehicle->set_altitude(Alt);
    }
};

class LA_MsgHandler_RCOU : public LA_MsgHandler {
public:
    LA_MsgHandler_RCOU(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(name, f, analyze, vehicle) {
        // FIXME:
        if (find_field_info("Ch1")) {
            _analyze->add_data_source("SERVO_OUTPUT", "RCOU.Ch1");
            _analyze->add_data_source("SERVO_OUTPUT", "RCOU.Ch2");
            _analyze->add_data_source("SERVO_OUTPUT", "RCOU.Ch3");
            _analyze->add_data_source("SERVO_OUTPUT", "RCOU.Ch4");
            if (find_field_info("Ch5")) {
                _analyze->add_data_source("SERVO_OUTPUT", "RCOU.Ch5");
                _analyze->add_data_source("SERVO_OUTPUT", "RCOU.Ch6");
                _analyze->add_data_source("SERVO_OUTPUT", "RCOU.Ch7");
                _analyze->add_data_source("SERVO_OUTPUT", "RCOU.Ch8");
            }
        } else {
            _analyze->add_data_source("SERVO_OUTPUT", "RCOU.Chan1");
            _analyze->add_data_source("SERVO_OUTPUT", "RCOU.Chan2");
            _analyze->add_data_source("SERVO_OUTPUT", "RCOU.Chan3");
            _analyze->add_data_source("SERVO_OUTPUT", "RCOU.Chan4");
            if (find_field_info("Chan5")) {
                _analyze->add_data_source("SERVO_OUTPUT", "RCOU.Chan5");
                _analyze->add_data_source("SERVO_OUTPUT", "RCOU.Chan6");
                _analyze->add_data_source("SERVO_OUTPUT", "RCOU.Chan7");
                _analyze->add_data_source("SERVO_OUTPUT", "RCOU.Chan8");
            }
        } 
    };

    void xprocess(const uint8_t *msg) override {
        char label[4] = "Chx";
        char longlabel[6] = "Chanx";
        uint16_t value;
        for (uint8_t i=1; i<8; i++) {
            label[2] = '0' + i;
            longlabel[4] = '0' + i;
            if (!field_value(msg, label, value) &&
                !field_value(msg, longlabel, value)) {
                break;
            }
            // ::fprintf(stderr, "%d=%d\n", i, value);
            _vehicle->set_servo_output(i, value);
        }
    }
};

class LA_MsgHandler_STAT : public LA_MsgHandler {
public:
    LA_MsgHandler_STAT(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(name, f, analyze, vehicle) {
    }

    void xprocess(const uint8_t *msg) override;

private:
    bool have_added_STAT = false;
};

#endif
