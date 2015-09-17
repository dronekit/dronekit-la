#ifndef _ANALYZER_VEHICLE
#define  _ANALYZER_VEHICLE

#include <stdint.h>
#include <map>

#include "mavlink/c_library/ardupilotmega/mavlink.h"
#include "mavlink/c_library/common/mavlink.h"

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

    class Position {
    public:
        void set_alt(uint64_t T, float alt) {
            _alt = alt;
            _alt_modtime = T;
        }
        float alt() { return _alt; };
        uint64_t alt_modtime() { return _alt_modtime; };

        void set_lat(uint64_t T, double lat) {
            _lat = lat;
            _lat_modtime = T;
        }
        double lat() { return _lat; };
        uint64_t lat_modtime() { return _lat_modtime; };

        void set_lon(uint64_t T, double lon) {
            _lon = lon;
            _lon_modtime = T;
        }
        double lon() { return _lon; };
        uint64_t lon_modtime() { return _lon_modtime; };

//        double distance_to(Position otherpos);
        double horizontal_distance_to(Position otherpos);

    private:
        double _lat;
        double _lon;
        float _alt; // relative, in metres
        uint64_t _alt_modtime;
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
        void set_alt(uint64_t T, float alt) { _position.set_alt(T, alt); }
        void set_lat(uint64_t T, double lat) { _position.set_lat(T, lat); }
        void set_lon(uint64_t T, double lon) { _position.set_lon(T, lon); }
        float alt() { return _position.alt(); }
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


    template <typename packettype>
    class PacketHistory {
    public:
        PacketHistory() :
            next(0),
            count(0)
            { }
        void packet(packettype &packet) {
            memcpy(&packets[next++], &packet, sizeof(packet));
            if (next >= size) {
                next = 0;
            }
            if (count < size) {
                count++;
            }
        }
    private:
        static const uint8_t size = 20;
        uint8_t next;
        uint8_t count;
        packettype packets[size];
    };
    
class Base {
public:
    Base() { }
    virtual ~Base() { }

    // vehicle state information
    virtual bool is_flying() { return false; }
    void exceeding_angle_max() const;

    // arming 
    virtual bool is_armed() { return _armed; };
    virtual void set_armed(bool value) {  _armed = value; };

    // vehicle type
    enum vehicletype_t {
        invalid = 0,
        copter = 17
    };
    virtual vehicletype_t vehicletype() {
        return invalid;
    }

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
    float param(const char *name) { return _param[name]; };
    uint16_t param_count() { return _param.size(); };
    bool param(const char *name, float &ret);
    bool param_seen(const char *name) const;
    uint64_t param_modtime(const char *name) { return _param_modtime[name]; }
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

    void set_alt(float alt) { pos().set_alt(T(), alt); }
    float alt() { return pos().alt(); };
    uint64_t alt_modtime() { return pos().alt_modtime(); }

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

    Attitude& att() { return _att; };
    Position& pos() { return _pos; };

protected:
    AV_Nav& nav() { return _nav; };

    bool _armed = false;

    std::map<const std::string, float> _param;
    std::map<const std::string, uint64_t> _param_modtime;
    std::map<const std::string, float> _param_defaults = {
    };
    
    Attitude _att = { };
    Position _pos = { };
    AV_Nav _nav = { };
    
private:
    uint64_t _T = 0;

    vehicletype_t _vehicletype = invalid;

    Battery _battery;

    std::map<const std::string, PositionEstimate*> _position_estimates;
    std::map<const std::string, AttitudeEstimate*> _attitude_estimates;
    
    PacketHistory<mavlink_heartbeat_t> history_heartbeat;
    PacketHistory<mavlink_nav_controller_output_t> history_nav_controller_output;
    PacketHistory<mavlink_servo_output_raw_t> history_servo_output_raw;

}; // end class

}; // end namespace

#endif
