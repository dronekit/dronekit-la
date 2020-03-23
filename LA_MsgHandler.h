/**
 * @file
 * @author Peter Barker <peter.barker@3drobotics.com>
 *
 * @section DESCRIPTION
 *
 * Objects to interpret information out of DataFlash messages and into
 * the Vehicle model
 */

#ifndef AP_LA_MSGHANDLER_H
#define AP_LA_MSGHANDLER_H

#include "MsgHandler.h"

#include "analyze.h"
#include "analyzervehicle.h"
#include "analyzervehicle_copter.h"
#include "analyzervehicle_plane.h"

#include <string.h>

/// @brief Base class for dataflash message handlers
///
/// Extend this based on the type of the DataFlash message you wish to
/// take data from
class LA_MsgHandler : public MsgHandler {
public:

    /// @brief construct a DataFlash Message Handler
    /// @param name Name by which this message handler is known; e.g. GPS2
    /// @param f DataFlash log format corresponding to the DataFlash message type
    /// @param analyze Object which is responsible for coordinating analysis of vehicle state
    /// @param vehicle a vehicle whose state should be updated by the message handler
    /// @note evaluate_all is called on "analyze" after the vehicle state has been updated
    LA_MsgHandler(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        MsgHandler(f),
        _name(name),
        _analyze(analyze),
        _vehicle(vehicle)
        {
            // _analyze->add_data_source("SYSTEM_TIME", "SYSTEM_TIME.time_boot_ms");
        };

    /// @brief process a DataFlash message, update state, run analysis on new vehicle state
    /// @param msg a DataFlash message
    virtual void process(const uint8_t *msg) {
        if (!process_set_T(msg)) {
            return;
        }
        xprocess(msg);
        _analyze->evaluate_all();
    }

    /// @brief return the name of this Message Handler (e.g. GPS2)
    const std::string name() const { return _name; }

protected:
    std::string _name;
    Analyze *_analyze;
    AnalyzerVehicle::Base *&_vehicle;

    /// @brief Try to find a timestamp given a message
    /// @param msg a dataflash message
    /// @param[out] T a timestamp value to fill in
    /// @return true if a timestamp was found
    virtual bool find_T(const uint8_t *msg, uint64_t &T);
    /// @brief Try to set vehicle model timestamp from a given message
    /// @return true if timestamp was set
    bool process_set_T(const uint8_t *msg);
    /// @brief return the current vehicle timestamp
    uint64_t T() const { return _vehicle->T(); } // only valid after process_set_T

    /// @brief process a DataFlash message, update vehicle model appropriately
    virtual void xprocess(const uint8_t *msg) = 0;

    /// @brief convenience function producing an estimate to be used for state
    AnalyzerVehicle::AltitudeEstimate* altitude_estimate();
    /// @brief convenience function producing an estimate to be used for state
    AnalyzerVehicle::AttitudeEstimate* attitude_estimate();
    /// @brief convenience function producing an estimate to be used for state
    AnalyzerVehicle::PositionEstimate* position_estimate();
    /// @brief convenience function producing an estimate to be used for state
    AnalyzerVehicle::VelocityEstimate* velocity_estimate();
    /// @brief convenience function producing an estimate to be used for state
    AnalyzerVehicle::GPSInfo* gpsinfo();
    /// @brief convenience function producing IMU to be used for state
    AnalyzerVehicle::IMU* imu();
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
    LA_MsgHandler_AHR2(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle);
    void xprocess(const uint8_t *msg) override;

    bool canonical_for_position() { return _canonical_for_position; };
    void set_canonical_for_position(bool value) { _canonical_for_position = value; }
    bool canonical_for_origin() { return _canonical_for_origin; };
    void set_canonical_for_origin(bool value) { _canonical_for_origin = value; }

private:
    bool _canonical_for_position = true;
    bool _canonical_for_origin = true;
    bool _was_armed = false;
    bool _was_canonical_for_position = false;
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
    LA_MsgHandler_EKF1(std::string name, const struct log_Format &f,
                       Analyze *analyze, AnalyzerVehicle::Base *&vehicle);
    void xprocess(const uint8_t *msg) override;
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

class LA_MsgHandler_ERR : public LA_MsgHandler {
public:
    LA_MsgHandler_ERR(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle);

    void xprocess(const uint8_t *msg) override;
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

class LA_MsgHandler_IMU : public LA_MsgHandler {
public:

    LA_MsgHandler_IMU(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle);
    void xprocess(const uint8_t *msg) override;
};


class LA_MsgHandler_GPA : public LA_MsgHandler {

public:

    LA_MsgHandler_GPA(const std::string name, const std::string gps_name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(name, f, analyze, vehicle),
        _gps_name(gps_name) {
    }

    void xprocess(const uint8_t *msg) override;

private:

    bool have_added_GPA = false;
    const std::string _gps_name;
};


class LA_MsgHandler_GPS : public LA_MsgHandler {
public:
    bool find_T(const uint8_t *msg, uint64_t &T) override;

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
        _analyze->add_data_source(string_format("GPSINFO_FIXTYPE_%s",name.c_str()),
            string_format("%s.Status",name.c_str()));

        _analyze->add_data_source(string_format("VELOCITY_ESTIMATE_%s",name.c_str()), string_format("%s.Spd",name.c_str()));
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

class LA_MsgHandler_MAG : public LA_MsgHandler {
public:
    LA_MsgHandler_MAG(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(name, f, analyze, vehicle) {
        const char * cname = name.c_str();
        _analyze->add_data_source(string_format("MAGNETIC_FIELD_STRENGTH_%s", cname), string_format("%s.MagX", cname));
        _analyze->add_data_source(string_format("MAGNETIC_FIELD_STRENGTH_%s", cname), string_format("%s.MagY", cname));
        _analyze->add_data_source(string_format("MAGNETIC_FIELD_STRENGTH_%s", cname), string_format("%s.MagZ", cname));
    };

    void xprocess(const uint8_t *msg) override;
};


class LA_MsgHandler_MODE : public LA_MsgHandler {
public:
    LA_MsgHandler_MODE(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(name, f, analyze, vehicle) {
        _analyze->add_data_source("MODE", "MODE.Mode");
    };

    void xprocess(const uint8_t *mode) override;
};


class LA_MsgHandler_MSG : public LA_MsgHandler {
public:
    LA_MsgHandler_MSG(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(name, f, analyze, vehicle) {
        _analyze->add_data_source("VEHICLE_DEFINITION", "MSG.Message");
    };

    void xprocess(const uint8_t *msg) override;
};

class LA_MsgHandler_NKF1 : public LA_MsgHandler {
public:
    LA_MsgHandler_NKF1(std::string name, const struct log_Format &f,
                       Analyze *analyze, AnalyzerVehicle::Base *&vehicle);
    void xprocess(const uint8_t *msg) override;
};

class LA_MsgHandler_NKF4 : public LA_MsgHandler {
public:
    LA_MsgHandler_NKF4(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(name, f, analyze, vehicle) {
        _analyze->add_data_source("NKF_FLAGS", "NKF4.SS");
        _analyze->add_data_source("NKF_VARIANCES_velocity_variance", "NKF4.SV");
        _analyze->add_data_source("NKF_VARIANCES_pos_horiz_variance", "NKF4.SP");
        _analyze->add_data_source("NKF_VARIANCES_pos_vert_variance", "NKF4.SH");
        _analyze->add_data_source("NKF_VARIANCES_pos_compass_variance", "NKF4.SM");
        _analyze->add_data_source("NKF_VARIANCES_terrain_alt_variance", "NKF4.SVT");
    };
    void xprocess(const uint8_t *msg) override {
        _vehicle->nkf_set_variance("velocity", require_field_uint16_t(msg, "SV") / (double)100.0f);
        _vehicle->nkf_set_variance("pos_horiz", require_field_uint16_t(msg, "SP") / (double)100.0f);
        _vehicle->nkf_set_variance("pos_vert", require_field_uint16_t(msg, "SH") / (double)100.0f);
        _vehicle->nkf_set_variance("compass", require_field_uint16_t(msg, "SM") / (double)100.0f);
        _vehicle->nkf_set_variance("terrain_alt", require_field_uint16_t(msg, "SVT") / (double)100.0f);

        // SS came in somewhere between Copter 3.2.1 and 3.3rc9
        // FIXME: this will always be present
        uint32_t ss = 0; // this was 16 bits before 2020-ish
        if (field_value(msg, "SS", ss)) {
            _vehicle->nkf_set_flags(ss);
        }
    }
};

class LA_MsgHandler_XKF1 : public LA_MsgHandler {
public:
    LA_MsgHandler_XKF1(std::string name, const struct log_Format &f,
                       Analyze *analyze, AnalyzerVehicle::Base *&vehicle);
    void xprocess(const uint8_t *msg) override;
};

class LA_MsgHandler_XKF4 : public LA_MsgHandler {
public:
    LA_MsgHandler_XKF4(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(name, f, analyze, vehicle) {
        _analyze->add_data_source("XKF_FLAGS", "XKF4.SS");
        _analyze->add_data_source("XKF_VARIANCES_velocity_variance", "XKF4.SV");
        _analyze->add_data_source("XKF_VARIANCES_pos_horiz_variance", "XKF4.SP");
        _analyze->add_data_source("XKF_VARIANCES_pos_vert_variance", "XKF4.SH");
        _analyze->add_data_source("XKF_VARIANCES_pos_compass_variance", "XKF4.SM");
        _analyze->add_data_source("XKF_VARIANCES_terrain_alt_variance", "XKF4.SVT");
    };
    void xprocess(const uint8_t *msg) override {
        _vehicle->xkf_set_variance("velocity", require_field_uint16_t(msg, "SV") / (double)100.0f);
        _vehicle->xkf_set_variance("pos_horiz", require_field_uint16_t(msg, "SP") / (double)100.0f);
        _vehicle->xkf_set_variance("pos_vert", require_field_uint16_t(msg, "SH") / (double)100.0f);
        _vehicle->xkf_set_variance("compass", require_field_uint16_t(msg, "SM") / (double)100.0f);
        _vehicle->xkf_set_variance("terrain_alt", require_field_uint16_t(msg, "SVT") / (double)100.0f);

        // FIXME: this will always be present
        uint32_t ss = 0; // this was 16 bits before 2020-ish
        if (field_value(msg, "SS", ss)) {
            _vehicle->xkf_set_flags(ss);
        }
    }
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
        uint8_t origin_type = require_field_uint8_t(msg, "Type");
        if (origin_type != 0) { // ekf origin
            return;
        }
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

class LA_MsgHandler_PM : public LA_MsgHandler {
public:
    LA_MsgHandler_PM(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(name, f, analyze, vehicle) {
        _analyze->add_data_source("AUTOPILOT_SCHEDULING", "PM.NLon");
        _analyze->add_data_source("AUTOPILOT_SCHEDULING", "PM.NLoop");
        _analyze->add_data_source("AUTOPILOT_SCHEDULING", "PM.MaxT");
    };

    void xprocess(const uint8_t *msg) override;
};


class LA_MsgHandler_POWR : public LA_MsgHandler {
public:
    LA_MsgHandler_POWR(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(name, f, analyze, vehicle) {
        _analyze->add_data_source("AUTOPILOT_VCC", "POWR.Vcc");
    };

    void xprocess(const uint8_t *msg) override;
};


class LA_MsgHandler_POS : public LA_MsgHandler {
public:
    LA_MsgHandler_POS(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(name, f, analyze, vehicle) {
        _analyze->add_data_source("POSITION_ESTIMATE_POS", "POS.Lat");
        _analyze->add_data_source("POSITION_ESTIMATE_POS", "POS.Lng");
        _analyze->add_data_source("ALTITUDE", "POS.Alt");
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


class LA_MsgHandler_RATE : public LA_MsgHandler {
public:
    LA_MsgHandler_RATE(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle);
    void xprocess(const uint8_t *msg) override;
};


class LA_MsgHandler_RCOU : public LA_MsgHandler {
public:
    LA_MsgHandler_RCOU(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(name, f, analyze, vehicle) {
        // FIXME:
        if (find_field_info("C1")) {
            _analyze->add_data_source("SERVO_OUTPUT", "RCOU.C1");
            _analyze->add_data_source("SERVO_OUTPUT", "RCOU.C2");
            _analyze->add_data_source("SERVO_OUTPUT", "RCOU.C3");
            _analyze->add_data_source("SERVO_OUTPUT", "RCOU.C4");
            if (find_field_info("C5")) {
                _analyze->add_data_source("SERVO_OUTPUT", "RCOU.C5");
                _analyze->add_data_source("SERVO_OUTPUT", "RCOU.C6");
                _analyze->add_data_source("SERVO_OUTPUT", "RCOU.C7");
                _analyze->add_data_source("SERVO_OUTPUT", "RCOU.C8");
            }
        } else if (find_field_info("Ch1")) {
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
        char lbl[3] = "Cx";
        char label[4] = "Chx";
        char longlabel[6] = "Chanx";
        uint16_t value;
        for (uint8_t i=1; i<=8; i++) {
            lbl[1] = '0' + i;
            label[2] = '0' + i;
            longlabel[4] = '0' + i;
            if (!field_value(msg, lbl, value) &&
                !field_value(msg, label, value) &&
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


class LA_MsgHandler_UBX3 : public LA_MsgHandler {

public:

    LA_MsgHandler_UBX3(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(name, f, analyze, vehicle) {
    }

    void xprocess(const uint8_t *msg) override;

private:

    bool have_added_UBX3 = false;
};


class LA_MsgHandler_VIBE : public LA_MsgHandler {
public:
    LA_MsgHandler_VIBE(std::string name, const struct log_Format &f, Analyze *analyze, AnalyzerVehicle::Base *&vehicle) :
        LA_MsgHandler(name, f, analyze, vehicle) {
    }

    void xprocess(const uint8_t *msg) override;

private:
    bool have_added_VIBE = false;
};

#endif
