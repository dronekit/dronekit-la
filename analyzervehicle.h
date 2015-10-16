#ifndef _ANALYZER_VEHICLE
#define  _ANALYZER_VEHICLE

#include <stdint.h>
#include <map>

#include "mavlink/c_library/ardupilotmega/mavlink.h"
#include "mavlink/c_library/common/mavlink.h"

#include "analyzer_util.h"
#include "Vector3f.h"

namespace AnalyzerVehicle {

    // Attitude should be the best guess as to what the vehicle's
    // status is - typicall the POS message from dataflash, for
    // example
    class Attitude {
    public:
       
        float roll() const { return _att[0]; };
        float pitch() const { return _att[1]; };
        float yaw() const { return _att[2]; };
        uint64_t roll_modtime() const {
            return get_att_modtime(0);
        }
        uint64_t pitch_modtime() const {
            return get_att_modtime(1);
        }
        uint64_t yaw_modtime() const {
            return get_att_modtime(2);
        }
        void set_roll(uint64_t T, float roll) {
            _set_att_attrib(0, T, roll);
        }
        void set_pitch(uint64_t T, float pitch) {
            _set_att_attrib(1, T, pitch);
        }
        void set_yaw(uint64_t T, float yaw) {
            _set_att_attrib(2, T, yaw);
        }

    private:
        // all in degrees:
        float _att[3];
        uint64_t _att_modtime[3];

        void _set_att_attrib(uint8_t offset, uint64_t T, float value) {
            _att[offset] = value;
            _att_modtime[offset] = T;
        }
        uint64_t get_att_modtime(uint8_t offset) const {
            return _att_modtime[offset];
        }
};

    class Altitude {
    public:
        void set_alt(uint64_t T, double alt) {
            _alt = alt;
            _alt_modtime = T;
        }
        double alt() const {
            return _alt;
        }
        uint64_t alt_modtime() const {
            return _alt_modtime;
        }

    private:
        double _alt;
        uint64_t _alt_modtime;
    };

    class AltitudeEstimate {
    public:
        AltitudeEstimate(const std::string name) :
            _name(name)
            { }
        AltitudeEstimate() :
            _name(NULL),
            _altitude({})
            { }
        const std::string name() { return _name; }
        const Altitude altitude() { return _altitude; }
        void set_alt(uint64_t T, float alt) { _altitude.set_alt(T, alt); }
        float alt() { return _altitude.alt(); }

    private:
        const std::string _name;
        Altitude _altitude = { };
    };

    class Position {
    public:
        void set_lat(uint64_t T, double lat) {
            _lat = lat;
            _lat_modtime = T;
        }
        double lat() const { return _lat; };
        uint64_t lat_modtime() const { return _lat_modtime; };

        void set_lon(uint64_t T, double lon) {
            _lon = lon;
            _lon_modtime = T;
        }
        double lon() const { return _lon; };
        uint64_t lon_modtime() const { return _lon_modtime; };

//        double distance_to(Position otherpos);
        double horizontal_distance_to(Position otherpos) const;

        bool is_zero_zero() {
            return (is_zero(lat()) && is_zero(lon()));
        }

    private:
        double _lat;
        double _lon;
        uint64_t _lat_modtime;
        uint64_t _lon_modtime;
    };

    class EKF {
    public:
        void set_variance(uint64_t T, std::string name, double value) {
            _variances[name] = value;
            _variances_T[name] = T;
        }
        uint64_t variance_T(std::string name) {
            return _variances_T[name];
        }
            
        void set_flags(uint64_t T, uint16_t flags) {
            _flags = flags;
            _flags_T = T;
        }
        uint16_t flags() {
            return _flags;
        }
        uint64_t flags_T() {
            return _flags_T;
        }
        std::map<const std::string, double> variances() {
            return _variances;
        }
    private:
        std::map<const std::string, double> _variances;
        std::map<const std::string, uint64_t> _variances_T;

        uint16_t _flags = 0;
        uint64_t _flags_T = 0;
    };
    
    // class AV_PosNED {
    // public:
    //     uint16_t N;
    //     uint16_t E;
    //     uint16_t D;
    // };

    class AttitudeEstimate {
    public:
        AttitudeEstimate(const std::string name) :
            _name(name)
            { }
        AttitudeEstimate() :
            _name(NULL),
            _attitude({})
            { }
        const std::string name() { return _name; }
        const Attitude attitude() { return _attitude; }
        void set_roll(uint64_t T, double roll) { _attitude.set_roll(T, roll); }
        void set_pitch(uint64_t T, double pitch) { _attitude.set_pitch(T, pitch); }
        void set_yaw(uint64_t T, double yaw) { _attitude.set_yaw(T, yaw); }
        double roll() { return _attitude.roll(); }
        double pitch() { return _attitude.pitch(); }
        double yaw() { return _attitude.yaw(); }

    private:
        const std::string _name;
        Attitude _attitude;
    };

    class PositionEstimate {
    public:
        PositionEstimate(const std::string name) :
            _name(name)
            { }
        PositionEstimate() :
            _name(NULL),
            _position({})
            { }
        const std::string name() { return _name; }
        const Position position() { return _position; }
        void set_lat(uint64_t T, double lat) { _position.set_lat(T, lat); }
        void set_lon(uint64_t T, double lon) { _position.set_lon(T, lon); }
        double lat() { return _position.lat(); }
        double lon() { return _position.lon(); }

    private:
        const std::string _name;
        Position _position = { };
    };

    class Battery {
    public:
        float _remaining = 0; // percent
        uint64_t _remaining_modtime = 0;

        // uint32_t _capacity; // mAh
        // uint64_t _capacity_modtime = 0;

        bool _failsafe_event = 0;
        uint64_t _failsafe_event_T = 0;
    };
    
    class AV_Nav {
    public:
        void set_desroll(uint64_t T, float roll) {
            _des[0] = roll;
            _modtimes[0] = T;
        }
        float desroll() { return _des[0]; }

        void set_despitch(uint64_t T, float pitch) {
            _des[1] = pitch;
            _modtimes[1] = T;
        }
        float despitch() { return _des[1]; }

        void set_desyaw(uint64_t T, float yaw) {
            _des[2] = yaw;
            _modtimes[2] = T;
        }
        float desyaw() { return _des[2]; }

    private:
        float _des[3];
        uint64_t _modtimes[3];
    };


    class GPSInfo {
    public:
        GPSInfo(std::string name) { _name = name; }
        const std::string name() { return _name; }

        uint8_t fix_type() { return _fix_type; }
        void set_fix_type(uint8_t fix_type) { _fix_type = fix_type; }

        double hdop() { return _hdop; }
        void set_hdop(double hdop) { _hdop = hdop; }

        uint8_t satellites() { return _satellites_visible; }
        void set_satellites(uint8_t satellites) { _satellites_visible = satellites; }
        
    private:
        std::string _name;

        double _hdop = 0;
        uint8_t _satellites_visible = 0;
        uint8_t _fix_type = 0;        
    };


    // template <typename packettype>
    // class PacketHistory {
    // public:
    //     PacketHistory() :
    //         next(0),
    //         count(0)
    //         { }
    //     void packet(packettype &packet) {
    //         memcpy(&packets[next++], &packet, sizeof(packet));
    //         if (next >= size) {
    //             next = 0;
    //         }
    //         if (count < size) {
    //             count++;
    //         }
    //     }
    // private:
    //     static const uint8_t size = 20;
    //     uint8_t next;
    //     uint8_t count;
    //     packettype packets[size];
    // };
    
class Base {
public:
    Base() { }
    virtual ~Base() { }

    virtual const std::string typeString() const { return "Base"; }

    uint64_t time_since_boot() {
        return _time_since_boot;
    }
    uint64_t time_since_boot_T() {
        return _time_since_boot_T;
    }
    void set_time_since_boot(const uint64_t time_since_boot) {
        _time_since_boot = time_since_boot;
        _time_since_boot_T = T();
    }
    
    // vehicle state information
    virtual bool is_flying() { return false; }
    void exceeding_angle_max() const;

    // arming 
    virtual bool is_armed() { return _armed; };
    virtual void set_armed(bool value) {  _armed = value; };

    // vehicle type
    enum vehicletype_t {
        invalid = 0,
        copter = 17,
        plane = 19,
    };
    bool vehicletype_is_forced() { return _vehicletype_is_forced; }
    void set_vehicletype_is_forced(bool value) { _vehicletype_is_forced = value; }
    virtual vehicletype_t vehicletype() {
        return invalid;
    }

    static void switch_vehicletype(Base *&_vehicle, vehicletype_t newtype);

    // EKF
    void ekf_set_variance(const char *name, double value) {
        _ekf.set_variance(T(), name, value);
    }
    uint64_t ekf_variance_T(std::string name) {
        return _ekf.variance_T(name);
    }
    void ekf_set_flags(uint16_t flags) {
        _ekf.set_flags(T(), flags);
    }
    uint16_t ekf_flags() {
        return _ekf.flags();
    }
    uint64_t ekf_flags_T() {
        return _ekf.flags_T();
    }
    std::map<const std::string, double> ekf_variances() {
        return _ekf.variances();
    }

    // take_state copies state from one vehicle to another
    void take_state(Base *old);
    
    // Parameters
    float param(const std::string name) { return _param[name]; };
    uint16_t param_count() { return _param.size(); };
    bool param(const char *name, float &ret);
    bool param_seen(const std::string name) const;
    uint64_t param_modtime(const std::string name) { return _param_modtime[name]; }
    void param_set(const char *name, const float value);
    virtual bool param_default(const char *name, float &ret);
    bool param_with_defaults(const char *name, float &ret);

    // servo output
    void set_servo_output(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4, uint16_t ch5, uint16_t ch6, uint16_t ch7, uint16_t ch8);
    void set_servo_output(uint8_t channel_number, uint16_t value);

    // FIXME: scope
    uint16_t _servo_output[9] = { }; // indexed from 1
    EKF _ekf;

    // universe time
    void set_T(const uint64_t time_us);
    uint64_t T() { return _T; }

    // attitude and position information
    float roll() { return att().roll(); }
    float pitch() { return att().pitch(); }
    float yaw() { return att().yaw(); }

    uint64_t roll_modtime() { return att().roll_modtime(); }
    uint64_t pitch_modtime() { return att().pitch_modtime(); }
    uint64_t yaw_modtime() { return att().yaw_modtime(); }

    void set_roll(float roll) { att().set_roll(T(), roll); }
    void set_pitch(float roll) { att().set_pitch(T(), roll); }
    void set_yaw(float roll) { att().set_yaw(T(), roll); }

    void set_desroll(float roll) { nav().set_desroll(T(), roll); };
    void set_despitch(float pitch) { nav().set_despitch(T(), pitch); };
    void set_desyaw(float yaw) { nav().set_desyaw(T(), yaw); };

    float desroll() { return nav().desroll(); }
    float despitch() { return nav().despitch(); }
    float desyaw() { return nav().desyaw(); }

    void set_altitude(float value) {
        // pos().set_alt(T(), value);
        alt().set_alt(T(), value);
    }
    float altitude() { return alt().alt(); };
    uint64_t alt_modtime() { return alt().alt_modtime(); }

    void set_lat(double lat) { pos().set_lat(T(), lat); }
    void set_lon(double lon) { pos().set_lon(T(), lon); }
    double lat() { return pos().lat(); }
    double lon() { return pos().lon(); }

    PositionEstimate *position_estimate(const std::string name) {
        if (_position_estimates.count(name) == 0) {
            _position_estimates[name] = new PositionEstimate(name);
        }
        return _position_estimates[name];
    };

    AttitudeEstimate *attitude_estimate(const std::string name) {
        if (_attitude_estimates.count(name) == 0) {
            _attitude_estimates[name] = new AttitudeEstimate(name);
        }
        return _attitude_estimates[name];
    };

    AltitudeEstimate *altitude_estimate(const std::string name) {
        if (_altitude_estimates.count(name) == 0) {
            _altitude_estimates[name] = new AltitudeEstimate(name);
        }
        return _altitude_estimates[name];
    };

    
    std::map<const std::string, bool> sensors_health() {
        return _sensors_health;
    }
    void sensor_set_healthy(std::string name, bool value) {
        _sensors_health[name] = value;
    }

    // battery
    void set_battery_remaining(float percent) {
        _battery._remaining = percent;
        _battery._remaining_modtime = T();
    }
    float battery_remaining() {
        return _battery._remaining;
    }
    uint64_t battery_remaining_T() {
        return _battery._remaining_modtime;
    }
    void set_battery_in_failsafe(bool in_failsafe) {
        _battery._failsafe_event = in_failsafe;
        _battery._failsafe_event_T = T();
    }
    bool battery_in_failsafe() {
        return _battery._failsafe_event;
    }
    uint64_t battery_in_failsafe_T() {
        return _battery._failsafe_event_T;
    }

    const std::map<const std::string, AttitudeEstimate*> &attitude_estimates() {
        return _attitude_estimates;
    }
    const std::map<const std::string, PositionEstimate*> &position_estimates() {
        return _position_estimates;
    }
    const std::map<const std::string, AltitudeEstimate*> &altitude_estimates() {
        return _altitude_estimates;
    }

    // not really sure this belongs here; possibly move this out if we
    // ever move to a "state of the universe" object for the analyzers
    // rather than just a vehicle
    const std::map<const std::string, GPSInfo*> &gpsinfos() {
        return _gpsinfo;
    }

    GPSInfo *gpsinfo(const std::string name) {
        if (_gpsinfo.count(name) == 0) {
            _gpsinfo[name] = new GPSInfo(name);
        }
        return _gpsinfo[name];
    };

    // again, not sure if this Compass object should be in the vehicle
    // class here.
    class Compass {
    public:
        Compass(const std::string name) :
            _name(name)
            { }
        const std::string name() const { return _name; }
        Vector3f &field() { return _field; }
        uint64_t field_T() { // most recent timestamp of all components
            return _field_T;
        }
        void set_field_T(uint64_t field_T) { _field_T = field_T; }
    private:
        const std::string _name; // do we really want this?!
        Vector3f _field = { };
        uint64_t _field_T = 0;
    };

    const std::map<const std::string, Compass*> &compasses() {
        return _compasses;
    }

    Compass *compass(const std::string name) {
        if (_compasses.count(name) == 0) {
            _compasses[name] = new Compass(name);
        }
        return _compasses[name];
    };


    Attitude& att() { return _att; };
    Position& pos() { return _pos; };
    Altitude& alt() { return _alt; }; // absolute

    const Position &origin() const { return _origin; }
    double origin_lat() const { return _origin.lat(); }
    uint64_t origin_lat_T() const { return _origin.lat_modtime(); }
    double origin_lon() const { return _origin.lon(); }
    uint64_t origin_lon_T() const { return _origin.lon_modtime(); }
    void set_origin_lat(double value) { _origin.set_lat(T(),value); }
    void set_origin_lon(double value) { _origin.set_lon(T(),value); }
        
    Position& origin() { return _origin; }
    Altitude &origin_alt() { return _origin_altitude; }

    void set_origin_altitude(double value) { _origin_altitude.set_alt(T(),value); }
    double origin_altitude() { return _origin_altitude.alt(); }
    uint64_t origin_altitude_T() const { return _origin_altitude.alt_modtime(); }

    bool relative_alt(double &relative); // returns true if relative alt could be calculated

protected:
    AV_Nav& nav() { return _nav; };

    bool _armed = false;

    std::map<const std::string, float> _param;
    std::map<const std::string, uint64_t> _param_modtime;
    std::map<const std::string, float> _param_defaults = {};
    
    std::map<const std::string, bool> _sensors_health = {};

    Attitude _att = { };
    Position _pos = { };
    Altitude _alt = { };
    AV_Nav _nav = { };

    Position _origin = { };
    Altitude _origin_altitude = { };
private:
    bool _vehicletype_is_forced = false;
    uint64_t _T = 0;

    uint64_t _time_since_boot;
    uint64_t _time_since_boot_T = 0;

    vehicletype_t _vehicletype = invalid;

    Battery _battery;

    std::map<const std::string, PositionEstimate*> _position_estimates;
    std::map<const std::string, AttitudeEstimate*> _attitude_estimates;
    std::map<const std::string, AltitudeEstimate*> _altitude_estimates;

    std::map<const std::string, GPSInfo*> _gpsinfo;
    std::map<const std::string, Compass*> _compasses;

    // PacketHistory<mavlink_heartbeat_t> history_heartbeat;
    // PacketHistory<mavlink_nav_controller_output_t> history_nav_controller_output;
    // PacketHistory<mavlink_servo_output_raw_t> history_servo_output_raw;

}; // end class

} // end namespace

#endif
