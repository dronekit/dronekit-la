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

class LA_MsgHandler_ATT : public LA_MsgHandler {
public:
    LA_MsgHandler_ATT(const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(f, analyze, vehicle) { };
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
    }
};

class LA_MsgHandler_EKF4 : public LA_MsgHandler {
public:
    LA_MsgHandler_EKF4(const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(f, analyze, vehicle) { };
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
        LA_MsgHandler(f, analyze, vehicle) { };

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

class LA_MsgHandler_MSG : public LA_MsgHandler {
public:
    LA_MsgHandler_MSG(const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(f, analyze, vehicle) { };

    void set_vehicle_copter()
        {
            AnalyzerVehicle::Base *vehicle_old = _vehicle;
            AnalyzerVehicle::Copter *vehicle_new = new AnalyzerVehicle::Copter();
            vehicle_new->take_state(vehicle_old);
            _vehicle = vehicle_new;
            delete vehicle_old;
            _vehicle->set_vehicletype(AnalyzerVehicle::Base::vehicletype_t::copter);
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
        LA_MsgHandler(f, analyze, vehicle) { };

    void xprocess(const uint8_t *msg) override {
        const uint8_t namelen = 255;
        char name[namelen];

        memset(name, 0, namelen);
        require_field(msg, "Name", name, namelen);
        float value = require_field_float(msg, "Value");

        _vehicle->param_set(name, value);
    }

};

class LA_MsgHandler_RCOU : public LA_MsgHandler {
public:
    LA_MsgHandler_RCOU(const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(f, analyze, vehicle) { };

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
