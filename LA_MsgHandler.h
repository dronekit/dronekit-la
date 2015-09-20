#ifndef AP_LA_MSGHANDLER_H
#define AP_LA_MSGHANDLER_H

#include "MsgHandler.h"

#include "analyze.h"
#include "analyzervehicle.h"
#include "analyzervehicle_copter.h"

#include <string.h>

class LA_MsgHandler : public MsgHandler {
public:
    LA_MsgHandler(const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        MsgHandler(f),
        _analyze(analyze),
        _vehicle(vehicle)
        { };

    void process_set_T(const uint8_t *msg);
    uint64_t T() const { return _vehicle->T(); }

    virtual void xprocess(const uint8_t *msg) = 0;

    virtual void process(const uint8_t *msg) {
        process_set_T(msg);
        xprocess(msg);
        _analyze->evaluate_all();
    }

protected:
    Analyze *_analyze;
    AnalyzerVehicle::Base *&_vehicle;
};

class LA_MsgHandler_AHR2 : public LA_MsgHandler {
public:
    LA_MsgHandler_AHR2(const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(f, analyze, vehicle) {
        _analyze->add_data_source("ATTITUDE_ESTIMATE_AHR2", "AHR2.Roll");
        _analyze->add_data_source("ATTITUDE_ESTIMATE_AHR2", "AHR2.Pitch");
        _analyze->add_data_source("ATTITUDE_ESTIMATE_AHR2", "AHR2.Yaw");

        _analyze->add_data_source("POSITION_ESTIMATE_AHR2", "AHR2.Lat");
        _analyze->add_data_source("POSITION_ESTIMATE_AHR2", "AHR2.Lng");
    };
    void xprocess(const uint8_t *msg) override {
        int16_t Roll = require_field_int16_t(msg, "Roll");
        int16_t Pitch = require_field_int16_t(msg, "Pitch");
        float Yaw = require_field_float(msg, "Yaw");

        _vehicle->attitude_estimate("AHR2")->set_roll(T(), Roll/100.0f);
        _vehicle->attitude_estimate("AHR2")->set_pitch(T(), Pitch/100.0f);
        _vehicle->attitude_estimate("AHR2")->set_yaw(T(), Yaw-180);

        int32_t Lat = require_field_int32_t(msg, "Lat");
        int32_t Lng = require_field_int32_t(msg, "Lng");
        float Alt = require_field_float(msg, "Alt");

        _vehicle->position_estimate("AHR2")->set_lat(T(), Lat/10000000.0f);
        _vehicle->position_estimate("AHR2")->set_lon(T(), Lng/10000000.0f);
        _vehicle->altitude_estimate("AHR2")->set_alt(T(), Alt);

        if (canonical_for_position()) {
            _vehicle->set_lat(Lat/10000000.0f);
            _vehicle->set_lon(Lng/10000000.0f);
            _vehicle->set_altitude(Alt);
        }
    }

    bool canonical_for_position() { return _canonical_for_position; };
    void set_canonical_for_position(bool value) { _canonical_for_position = value; }

private:
    bool _canonical_for_position = true;
};

class LA_MsgHandler_ATT : public LA_MsgHandler {
public:
    LA_MsgHandler_ATT(const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(f, analyze, vehicle) {
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
    void xprocess(const uint8_t *msg) override {
        int16_t DesRoll = require_field_int16_t(msg, "DesRoll");
        int16_t Roll = require_field_int16_t(msg, "Roll");
        int16_t DesPitch = require_field_int16_t(msg, "DesPitch");
        int16_t Pitch = require_field_int16_t(msg, "Pitch");
        uint16_t DesYaw = require_field_uint16_t(msg, "DesYaw");
        uint16_t Yaw = require_field_uint16_t(msg, "Yaw");
    
        // _vehicle->set_roll(rad_to_deg(Roll/100.0f));
        // _vehicle->set_pitch(rad_to_deg(Pitch/100.0f));
        _vehicle->set_roll(Roll/100.0f);
        _vehicle->set_pitch(Pitch/100.0f);
        _vehicle->set_yaw(Yaw);

        _vehicle->set_desroll((float)DesRoll/100.0f);
        _vehicle->set_despitch((float)DesPitch/100.0f);
        _vehicle->set_desyaw(DesYaw);

        _vehicle->attitude_estimate("ATT")->set_roll(T(), Roll/100.0f);
        _vehicle->attitude_estimate("ATT")->set_pitch(T(), Pitch/100.0f);
        _vehicle->attitude_estimate("ATT")->set_yaw(T(), Yaw);
    }
};

class LA_MsgHandler_EKF1 : public LA_MsgHandler {
public:
    LA_MsgHandler_EKF1(const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(f, analyze, vehicle) {
        _analyze->add_data_source("ATTITUDE_ESTIMATE_EKF1", "EKF1.Roll");
        _analyze->add_data_source("ATTITUDE_ESTIMATE_EKF1", "EKF1.Pitch");
        _analyze->add_data_source("ATTITUDE_ESTIMATE_EKF1", "EKF1.Yaw");

        // these are all relative; need to work out an origin:
        // _analyze->add_data_source("POSITION_ESTIMATE_EKF1", "EKF1.PN");
        // _analyze->add_data_source("POSITION_ESTIMATE_EKF1", "EKF1.PE");
        // _analyze->add_data_source("POSITION_ESTIMATE_EKF1", "EKF1.PD");

    };
    void xprocess(const uint8_t *msg) override {
        int16_t Roll = require_field_int16_t(msg, "Roll");
        int16_t Pitch = require_field_int16_t(msg, "Pitch");
        float Yaw = require_field_float(msg, "Yaw");

        _vehicle->attitude_estimate("EKF1")->set_roll(T(), Roll/100.0f);
        _vehicle->attitude_estimate("EKF1")->set_pitch(T(), Pitch/100.0f);
        _vehicle->attitude_estimate("EKF1")->set_yaw(T(), Yaw-180);

        // these are all relative; need to work out an origin:
        // _vehicle->position_estimate("EKF1")->set_lat(T(), Roll/100.0f);
        // _vehicle->position_estimate("EKF1")->set_lon(T(), Pitch/100.0f);
        // _vehicle->position_estimate("EKF1")->set_alt(T(), Yaw-180);
    }
};

class LA_MsgHandler_EKF4 : public LA_MsgHandler {
public:
    LA_MsgHandler_EKF4(const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(f, analyze, vehicle) {
        _analyze->add_data_source("EKF_FLAGS", "EKF4.SS");
        _analyze->add_data_source("EKF_VARIANCE_velocity_variance", "EKF4.SV");
        _analyze->add_data_source("EKF_VARIANCE_pos_horiz_variance", "EKF4.SP");
        _analyze->add_data_source("EKF_VARIANCES_pos_vert_variance", "EKF4.SH");
        _analyze->add_data_source("EKF_VARIANCES_compass_variance", "EKF4.SMX");
        _analyze->add_data_source("EKF_VARIANCES_compass_variance", "EKF4.SMY");
        _analyze->add_data_source("EKF_VARIANCES_compass_variance", "EKF4.SMZ");
        _analyze->add_data_source("EKF_VARIANCES_terrain_alt_variance", "EKF4.SVT");
    };
    void xprocess(const uint8_t *msg) override {
        _vehicle->ekf_set_variance("velocity", require_field_uint16_t(msg, "SV") / 100.0f);
        _vehicle->ekf_set_variance("pos_horiz", require_field_uint16_t(msg, "SP") / 100.0f);
        _vehicle->ekf_set_variance("pos_vert", require_field_uint16_t(msg, "SH") / 100.0f);
        _vehicle->ekf_set_variance("terrain_alt", require_field_uint16_t(msg, "SVT") / 100.0f);

        double m[3];
        m[0] = require_field_uint16_t(msg, "SMX") / 100.0f;
        m[1] = require_field_uint16_t(msg, "SMY") / 100.0f;
        m[2] = require_field_uint16_t(msg, "SMZ") / 100.0f;
    
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
    LA_MsgHandler_ERR(const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(f, analyze, vehicle) {
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
    LA_MsgHandler_EV(const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(f, analyze, vehicle) {
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
    LA_MsgHandler_GPS(const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(f, analyze, vehicle) {
        _analyze->add_data_source("POSITION_ESTIMATE_GPS", "GPS.Lat");
        _analyze->add_data_source("POSITION_ESTIMATE_GPS", "GPS.Lng");
    };
    void xprocess(const uint8_t *msg) override {
        int32_t Lat = require_field_int32_t(msg, "Lat");
        int32_t Lng = require_field_int32_t(msg, "Lng");
        int32_t Alt = require_field_int32_t(msg, "Alt");

        _vehicle->position_estimate("GPS")->set_lat(T(), Lat/10000000.0f);
        _vehicle->position_estimate("GPS")->set_lon(T(), Lng/10000000.0f);
        _vehicle->altitude_estimate("GPS")->set_alt(T(), Alt/100.0f);
    }
};

class LA_MsgHandler_GPS2 : public LA_MsgHandler {
public:
    LA_MsgHandler_GPS2(const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(f, analyze, vehicle) {
        _analyze->add_data_source("POSITION_ESTIMATE_GPS2", "GPS2.Lat");
        _analyze->add_data_source("POSITION_ESTIMATE_GPS2", "GPS2.Lng");
    };
    void xprocess(const uint8_t *msg) override {
        int32_t Lat = require_field_int32_t(msg, "Lat");
        int32_t Lng = require_field_int32_t(msg, "Lng");
        int32_t Alt = require_field_int32_t(msg, "Alt");

        _vehicle->position_estimate("GPS2")->set_lat(T(), Lat/10000000.0f);
        _vehicle->position_estimate("GPS2")->set_lon(T(), Lng/10000000.0f);
        _vehicle->altitude_estimate("GPS2")->set_alt(T(), Alt/100.0f);
    }
};

class LA_MsgHandler_MSG : public LA_MsgHandler {
public:
    LA_MsgHandler_MSG(const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(f, analyze, vehicle) {
        _analyze->add_data_source("VEHICLE_DEFINITION", "MSG.Message");
    };

    void set_vehicle_copter()
        {
            AnalyzerVehicle::Base *vehicle_old = _vehicle;
            AnalyzerVehicle::Copter *vehicle_new = new AnalyzerVehicle::Copter();
            vehicle_new->take_state(vehicle_old);
            _vehicle = vehicle_new;
            delete vehicle_old;
        }

    void xprocess(const uint8_t *msg) override {
        char msg_message[160];
        uint8_t msg_message_len = 160;
        require_field(msg, "Message", msg_message, msg_message_len);

        if (strstr(msg_message, "APM:Copter") || strstr(msg_message, "ArduCopter")) {
            set_vehicle_copter();
        }

        switch(_vehicle->vehicletype()) {
        case AnalyzerVehicle::Base::vehicletype_t::copter:
            if (strstr(msg_message, "Frame")) {
                ((AnalyzerVehicle::Copter*&)_vehicle)->set_frame(msg_message);
            }
            break;
        case AnalyzerVehicle::Base::vehicletype_t::invalid:
            ::fprintf(stderr, "unhandled message (%s)\n", msg_message);
            // abort();
        }
    }
};

class LA_MsgHandler_PARM : public LA_MsgHandler {
public:
    LA_MsgHandler_PARM(const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(f, analyze, vehicle) {
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
    LA_MsgHandler_POS(const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(f, analyze, vehicle) {
        _analyze->add_data_source("POSITION_ESTIMATE_POS", "POS.Lat");
        _analyze->add_data_source("POSITION_ESTIMATE_POS", "POS.Lng");
    };
    void xprocess(const uint8_t *msg) override {
        int32_t Lat = require_field_int32_t(msg, "Lat");
        int32_t Lng = require_field_int32_t(msg, "Lng");
        float Alt = require_field_float(msg, "Alt");

        _vehicle->position_estimate("POS")->set_lat(T(), Lat/10000000.0f);
        _vehicle->position_estimate("POS")->set_lon(T(), Lng/10000000.0f);
        _vehicle->altitude_estimate("POS")->set_alt(T(), Alt);

        _vehicle->set_lat(Lat/10000000.0f);
        _vehicle->set_lon(Lng/10000000.0f);
        _vehicle->set_altitude(Alt);
    }
};

class LA_MsgHandler_RCOU : public LA_MsgHandler {
public:
    LA_MsgHandler_RCOU(const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(f, analyze, vehicle) {
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

#endif
