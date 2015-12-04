#ifndef _ANALYZER_VEHICLE
#define  _ANALYZER_VEHICLE

#include <stdint.h>
#include <map>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "mavlink/c_library/ardupilotmega/mavlink.h"
#include "mavlink/c_library/common/mavlink.h"
#pragma GCC diagnostic pop

#include "analyzer_util.h"
#include "Vector3f.h"

namespace AnalyzerVehicle {

    /// @brief abstraction of a vehicle attitude
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

    /// @brief abstraction of a vehicle altitude
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

    /// @brief Altitude according to some sensor or algorithm
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

    /// @brief abstraction of a vehicle global position (2D)
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

    /// @brief abstraction of a vehicle velocity
    class Velocity {
    public:
        double size() {
            if (! is_equal(_velocity_scalar, -1.0f)) {
                return _velocity_scalar;
            }
            if (! is_equal(_velocity[2], -1.0f)) {
                return _velocity.len();
            }
            return sqrt(_velocity[0]*_velocity[0] + _velocity[1] * _velocity[1]);
        }

        void set_x(uint64_t T, double x) {
            _have_components = true;
            _velocity[0] = x;
            _velocity_modtime = T;
        }
        void set_y(uint64_t T, double y) {
            _have_components = true;
            _velocity[1] = y;
            _velocity_modtime = T;
        }
        void set_z(uint64_t T, double z) {
            _have_components = true;
            _velocity[2] = z;
            _velocity_modtime = T;
        }
        void set_scalar(uint64_t T, double scalar) {
            _velocity_scalar = scalar;
            _velocity_modtime = T;
        }
        uint64_t velocity_modtime() {
            return _velocity_modtime;
        }

    private:
        // only one of these two should be set:
        Vector3f _velocity = { };
        double _velocity_scalar = -1;
        bool _have_components = false;
        uint64_t _velocity_modtime = 0;
    };

    /// @brief state information for a base Extended Kalman Filter
    class EKF {
    public:
        void set_variance(uint64_t T, std::string name, double value) {
            _variances[name] = value;
            _variances_T[name] = T;
        }
        uint64_t variance_T(std::string name) const;
            
        void set_flags(uint64_t T, uint16_t flags) {
            _flags = flags;
            _flags_T = T;
        }
        uint16_t flags() const {
            return _flags;
        }
        uint64_t flags_T() const {
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

    /// @brief Attitude according to some sensor or algorithm
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

    /// @brief Position according to some sensor or algorithm
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

    /// @brief Information on a vehicle battery
    class Battery {
    public:
        float _remaining = 0; // percent
        uint64_t _remaining_modtime = 0;

        // uint32_t _capacity; // mAh
        // uint64_t _capacity_modtime = 0;

        bool _failsafe_event = 0;
        uint64_t _failsafe_event_T = 0;
    };
    
    /// @brief Information about the Navigation Controller's output
    class AV_Nav {
    public:
        void set_desroll(uint64_t T, float roll) {
            _des[0] = roll;
            _modtimes[0] = T;
        }
        float desroll() const { return _des[0]; }

        void set_despitch(uint64_t T, float pitch) {
            _des[1] = pitch;
            _modtimes[1] = T;
        }
        float despitch() const { return _des[1]; }

        void set_desyaw(uint64_t T, float yaw) {
            _des[2] = yaw;
            _modtimes[2] = T;
        }
        float desyaw() const { return _des[2]; }

    private:
        float _des[3];
        uint64_t _modtimes[3];
    };


    /// @brief A source of magnetic field information
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


    /// @brief A source of GPS information (typically output from a GPS device)
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

    /* may need to factor and subclass this for non-APM-on-PixHawk: */
    /// @brief Information about the AutoPilot itself
    class AutoPilot {
    public:
        uint16_t overruns() { return _overruns; }
        uint16_t overruns_T() { return _overruns_T; }
        void set_overruns(uint64_t T, uint16_t);

        uint16_t loopcount() { return _loopcount; }
        void set_loopcount(uint64_t T, uint16_t);

        uint64_t slices_max_T() { return _slices_max_T; }
        uint16_t slices_max() { return _slices_max; }
        void set_slices_max(uint64_t T, uint16_t);

        uint16_t slices_min() { return _slices_min; }
        void set_slices_min(uint64_t T, uint16_t);

        uint16_t slices_avg() { return _slices_avg; }
        void set_slices_avg(uint64_t T, uint16_t);

        uint16_t slices_stddev() { return _slices_stddev; }
        void set_slices_stddev(uint64_t T, uint16_t);
        
    private:
        uint16_t _overruns;
        uint64_t _overruns_T = 0;
        uint16_t _loopcount;
        uint64_t _loopcount_T = 0;
        uint16_t _slices_max;
        uint64_t _slices_max_T = 0;
        uint16_t _slices_min;
        uint64_t _slices_min_T = 0;
        uint16_t _slices_avg;
        uint64_t _slices_avg_T = 0;
        uint16_t _slices_stddev;
        uint64_t _slices_stddev_T = 0;
    };

    /// @brief Accelerometer and gyroscope information from onboard IMUs
    class IMU {
    public:
        IMU(const std::string name, uint64_t &T) :
            _name(name),
            _T(T)
            { }
        const std::string name() const { return _name; }
        uint64_t T() const { return _T; };
        Vector3f &acc() { return _acc; }
        Vector3f &gyr() { return _gyr; }
        uint64_t acc_T() { // most recent timestamp of all components
            return _acc_T;
        }
        uint64_t gyr_T() { // most recent timestamp of all components
            return _gyr_T;
        }
        void set_acc_T(uint64_t acc_T) { _acc_T = acc_T; }
        void set_gyr_T(uint64_t gyr_T) { _gyr_T = gyr_T; }

        void set_acc_clip_count(uint16_t count);

        uint64_t last_acc_clip_time() const;
        bool acc_is_clipping() const;

    private:
        const std::string _name; // do we really want this?!
        uint64_t &_T;

        Vector3f _acc = { };
        Vector3f _gyr = { };
        uint64_t _acc_T = 0;
        uint64_t _gyr_T = 0;

        uint64_t _acc_clip_count_T;
        uint16_t _acc_clip_count;
    };

/// @brief Base class from which all vehicles inherit
class Base {
public:
    Base() { }
    virtual ~Base() { }

    /// @brief returns a short string describing the vehicle
    /// @return a short string describing the vehicle e.g. "Copter"
    virtual const std::string typeString() const { return "Base"; }

    /// @brief time since autopilot boot (microseconds)
    /// @return time since autopilot boot (microseconds)
    uint64_t time_since_boot() const {
        return _time_since_boot;
    }
    /// @brief the timestamp at which the "time_since_boot()" timestamp was updated
    /// @details this timestamp may reflect time on a different system - for example, a MAVLink tlog often uses a Ground Control Station's concept of time
    /// @return timestamp that the autopilot boot time was last changed
    uint64_t time_since_boot_T() const {
        return _time_since_boot_T;
    }
    /// @brief set the number of microseconds the autopilot has been booted 
    /// @param time_since_boot microseconds since autopilot boot
    void set_time_since_boot(const uint64_t time_since_boot) {
        _time_since_boot = time_since_boot;
        _time_since_boot_T = T();
    }
    
    /// @brief evaluation of whether the vehicle is flying
    /// @return true if it is believed the vehicle is flying
    virtual bool is_flying() const { return false; }

    /// @brief vehicle arm status
    /// @return true if it believed the vehicle is armed
    virtual bool is_armed() const { return _armed; };
    /// @brief set the vehicle arm status
    /// @param value the new vehicle arm status
    virtual void set_armed(bool value) {  _armed = value; };

    /// @brief all the different vehicles we have specific tests for
    enum vehicletype_t {
        invalid = 0,
        copter = 17,
        plane = 19,
    };
    /// @brief vehicle type forced status
    /// @detail sometimes the vehicle type can not be determined from a log and must be forced
    /// @return true if the vehicle type has been forced
    bool vehicletype_is_forced() const { return _vehicletype_is_forced; }
    /// @brief indicate that the vehicle type has been forced
    /// @param value true if the vehicle type has been forced
    void set_vehicletype_is_forced(bool value) { _vehicletype_is_forced = value; }
    /// @brief vehicle type
    /// @detail until the vehicle type is determined, this returns invalid.  After the vehicle type is determined (e.g. from a log), this will return a specific value from the vehicletype_t enumeration indication the vehicle type.
    /// @return the current vehicle type
    virtual vehicletype_t vehicletype() const {
        return invalid;
    }

    /// @brief change the vehicle from one type to another
    /// @detail a base vehicle is used to gather information until we determine what vehicle type we are actually analysing.  This call is used to switch to a more-specific vehicle type
    /// @param _vehicle to change type of
    /// @newtype new vehicle type
    static void switch_vehicletype(Base *&_vehicle, vehicletype_t newtype);

    /// @brief set a variance value by name
    /// @param name name of variance to set (e.g. "pos_horiz")
    /// @param value new value of the variance
    void ekf_set_variance(const char *name, double value) {
        _ekf.set_variance(T(), name, value);
    }
    /// @brief variance modification time
    /// @param name name of variance to retrieve modification time for
    /// @return timestamp at which variance  was last changed (microseconds)
    uint64_t ekf_variance_T(std::string name) const {
        return _ekf.variance_T(name);
    }
    /// @brief set EKF status flags
    /// @param flags EKF self-assesment status flags
    void ekf_set_flags(uint16_t flags) {
        _ekf.set_flags(T(), flags);
    }
    /// @brief EKF status flags
    /// @return EKF self-assesment status flags
    uint16_t ekf_flags() {
        return _ekf.flags();
    }

    /// @brief EKF status flags modification time
    /// @return timestamp at which the EKF self-assesment flags were last changed (microseconds)
    uint64_t ekf_flags_T() const {
        return _ekf.flags_T();
    }
    /// @brief all ekf variances
    /// @return returns all ekf variances as a map from name to value
    std::map<const std::string, double> ekf_variances() {
        return _ekf.variances();
    }

    /// @brief retrieve parameter value
    /// @param name parameter value to retrieve
    /// @detail this function will abort if the parameter has not been seen and no default is known
    /// @return parameter value from log or defaults
    float param(const std::string name) const;
    /// @brief retrieve parameter value
    /// @detail returns parameter value ONLY from input, not from defaults
    /// @param name parameter value to retrieve
    /// @param[out] parameter value
    /// @return true if the parameter has been seen in the log
    bool param(const char *name, float &ret) const;
    /// @brief number of parameters seen
    /// @return number of parameters seen in input
    uint16_t param_count() const { return _param.size(); };
    /// @brief parameter status
    /// @return true if the parameter has been seen in the log
    bool param_seen(const std::string name) const;
    /// @brief timestamp at which a parameter was last modified (microseconds)
    /// @param name parameter modification time to retrieve
    /// @return parameter modification time (microseconds)
    uint64_t param_modtime(const std::string name) const;
    /// @brief set a parameter
    /// @param name parameter to set
    /// @param value new value of parameter
    void param_set(const char *name, const float value);
    /// @brief return the default value for a parameter
    /// @detail default parameters are supplied so that some analysis can be done even in the absence of parameters in the logs
    /// @param name parameter to retirve default value for
    /// @param[out] ret value of parameter
    /// @return true if a default was found for the parameters
    virtual bool param_default(const char *name, float &ret) const;
    /// @brief retrieve a parameter value
    /// @detail returns the value last set for this parameter, or a default if it has not been set yet
    /// @param name parameter to retirve default value for
    /// @param[out] ret value of parameter
    /// @return true if the parameter value was found
    bool param_with_defaults(const char *name, float &ret) const;
    /// @brief retrieves a parameter value and abort()s if it not found
    /// @detail returns the value last set for this parameter, or a default if it has not been set yet
    /// @param name parameter to retirve default value for
    /// @return value of parm
    float require_param_with_defaults(const char *name) const;

    /// @brief set outputs for all servos
    /// @param ch1 current output for servo 1 (ppm)
    /// @param ch2 current output for servo 2 (ppm)
    /// @param ch3 current output for servo 3 (ppm)
    /// @param ch4 current output for servo 4 (ppm)
    /// @param ch5 current output for servo 5 (ppm)
    /// @param ch6 current output for servo 6 (ppm)
    /// @param ch7 current output for servo 7 (ppm)
    /// @param ch8 current output for servo 8 (ppm)
    void set_servo_output(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4, uint16_t ch5, uint16_t ch6, uint16_t ch7, uint16_t ch8);
    /// @brief set outputs for single servo
    /// @param channel_number servo to set value for
    /// @param value new value for servo output (ppm)
    void set_servo_output(uint8_t channel_number, uint16_t value);

    /// @brief return current output for a specific servo
    /// @param channel_number servo number
    /// @return current servo output (ppm)
    uint16_t servo_output(uint8_t channel_number) const {
        return _servo_output[channel_number];
    }
    
    /// @brief set timestamp distinguishing logging events
    /// @detail this may or may not be a real wall-clock time
    /// @param time_us new timestamp (microseconds)
    void set_T(const uint64_t time_us);
    /// @brief current timestamp
    /// @return current universe timestamp
    uint64_t T() { return _T; }

    /// @brief canonical vehicle attitude - roll
    /// @return canonical attitude - roll (degrees)
    float roll() const { return att().roll(); }
    /// @brief canonical vehicle attitude - pitch (degrees)
    /// @return canonical attitude - pitch
    float pitch() const { return att().pitch(); }
    /// @brief canonical vehicle attitude - yaw
    /// @return canonical attitude - yaw (degrees)
    float yaw() const { return att().yaw(); }

    /// @brief modification time - roll
    /// @return timestamp roll was last modified (microseconds)
    uint64_t roll_modtime() const { return att().roll_modtime(); }
    /// @brief modification time - pitch
    /// @return timestamp pitch was last modified (microseconds)
    uint64_t pitch_modtime() const { return att().pitch_modtime(); }
    /// @brief modification time - yaw
    /// @return timestamp yaw was last modified (microseconds)
    uint64_t yaw_modtime() const { return att().yaw_modtime(); }

    /// @brief set canonical vehicle atttiude - roll
    /// @param new canonical attitude - roll (degrees)
    void set_roll(float roll) { att().set_roll(T(), roll); }
    /// @brief set canonical vehicle atttiude - pitch
    /// @param new canonical attitude - pitch (degrees)
    void set_pitch(float roll) { att().set_pitch(T(), roll); }
    /// @brief set canonical vehicle atttiude - yaw
    /// @param new canonical attitude - yaw (degrees)
    void set_yaw(float roll) { att().set_yaw(T(), roll); }

    /// @brief set Navigation Controller desired attitude - roll
    /// @param roll new desired attitude - roll
    void set_desroll(float roll) { _nav.set_desroll(T(), roll); };
    /// @brief set Navigation Controller desired attitude - pitch
    /// @param roll new desired attitude - pitch
    void set_despitch(float pitch) { _nav.set_despitch(T(), pitch); };
    /// @brief set Navigation Controller desired attitude - yaw
    /// @param roll new desired attitude - yaw
    void set_desyaw(float yaw) { _nav.set_desyaw(T(), yaw); };

    /// @brief Navigation Controller desired attitude - roll
    float desroll() const { return nav().desroll(); }
    /// @brief Navigation Controller desired attitude - pitch
    float despitch() const { return nav().despitch(); }
    /// @brief Navigation Controller desired attitude - yaw
    float desyaw() const { return nav().desyaw(); }

    /// @brief set canonical vehicle altitude
    /// @param value new vehicle altitude
    void set_altitude(float value) {
        alt().set_alt(T(), value);
    }
    /// @brief canonical vehicle altitude
    /// @return the vehicle's canonical altitude
    float altitude() const { return alt().alt(); };
    /// @brief canonical vehicle altitude modification time
    /// @return timestamp the canonical vehicle altitude was modified (microseconds)
    uint64_t alt_modtime() const { return alt().alt_modtime(); }

    /// @brief set vehicle canonical latitude
    /// @param lat new vehicle canonical latitude
    void set_lat(double lat) { pos().set_lat(T(), lat); }
    /// @brief set vehicle canonical longitude
    /// @param lat new vehicle canonical latitude
    void set_lon(double lon) { pos().set_lon(T(), lon); }
    // @brief vehicle canonlical latitude
    // @return vehicle's canonical latitude
    double lat() { return pos().lat(); }
    // @brief vehicle canonlical longitude
    // @return vehicle's canonical longitude
    double lon() { return pos().lon(); }

    /// @brief retrieve named position estimate
    /// @param name name of position estimate to retrieve
    /// @return a position estimate
    PositionEstimate *position_estimate(const std::string name) {
        if (_position_estimates.count(name) == 0) {
            _position_estimates[name] = new PositionEstimate(name);
        }
        return _position_estimates[name];
    };

    /// @brief retrieve named attitude estimate
    /// @param name name of attitude estimate to retrieve
    /// @return a attitude estimate
    AttitudeEstimate *attitude_estimate(const std::string name) {
        if (_attitude_estimates.count(name) == 0) {
            _attitude_estimates[name] = new AttitudeEstimate(name);
        }
        return _attitude_estimates[name];
    };

    /// @brief retrieve named altitude estimate
    /// @param name name of altitude estimate to retrieve
    /// @return a altitude estimate
    AltitudeEstimate *altitude_estimate(const std::string name) {
        if (_altitude_estimates.count(name) == 0) {
            _altitude_estimates[name] = new AltitudeEstimate(name);
        }
        return _altitude_estimates[name];
    };

    /// @brief distance from canonical vehicle position to canonical origin
    /// @return distance from origin, or -1 if we just don't know
    double distance_from_origin();
    
    /// @brief hardware diagnostics
    /// @return map of sensor name to its boolean health
    std::map<const std::string, bool> sensors_health() {
        return _sensors_health;
    }
    /// @brief set the health status of a sensor by name
    /// @param name name of sensor
    /// @param value new health value of sensor
    void sensor_set_healthy(std::string name, bool value) {
        _sensors_health[name] = value;
    }

    /// @brief indicate amount of flight battery remaining
    /// @param percent percentage of charge remaining
    void set_battery_remaining(float percent) {
        _battery._remaining = percent;
        _battery._remaining_modtime = T();
    }
    /// @brief amount of battery remaining
    /// @return amount of battery remaining (percentage)
    float battery_remaining() {
        return _battery._remaining;
    }
    /// @brief battery remaining modification time
    /// @return timestamp when battery remaining was last modified (microseconds)
    uint64_t battery_remaining_T() {
        return _battery._remaining_modtime;
    }
    /// @brief indicate a battery in in failsafe
    /// @param in_failsafe true if battery is in failsafe
    void set_battery_in_failsafe(bool in_failsafe) {
        _battery._failsafe_event = in_failsafe;
        _battery._failsafe_event_T = T();
    }
    /// @brief battery failsafe status
    /// @return true if the battery is in failsafe
    bool battery_in_failsafe() {
        return _battery._failsafe_event;
    }
    /// @brief battery in failsafe modification time
    /// @return timestamp when battery failsafe was last modified (microseconds)
    uint64_t battery_in_failsafe_T() {
        return _battery._failsafe_event_T;
    }

    /// @brief attitude estimates
    /// @return map of attitude estimate name to attitude estimate
    const std::map<const std::string, AttitudeEstimate*> &attitude_estimates() {
        return _attitude_estimates;
    }
    /// @brief altitude estimates
    /// @return map of altitude estimate name to altitude estimate
    const std::map<const std::string, AltitudeEstimate*> &altitude_estimates() {
        return _altitude_estimates;
    }
    /// @brief position estimates
    /// @return map of position estimate name to position estimate
    const std::map<const std::string, PositionEstimate*> &position_estimates() {
        return _position_estimates;
    }

    /// @brief autopilot status information
    /// @return object containing autopilot status information
    AutoPilot &autopilot() {
        return _autopilot;
    }

    /// @brief set number of autopilot scheduler overruns
    /// @param scheduler overruns in last loopcount() loops
    void autopilot_set_overruns(uint16_t overruns) {
        _autopilot.set_overruns(T(), overruns);
    }
    /// @brief set number of loops for scheduler data
    /// @param count number of loops for scheduler data
    void autopilot_set_loopcount(uint16_t count) {
        _autopilot.set_loopcount(T(), count);
    }
    /// @brief maximum number of slices moved in a scheduler loop
    /// @param slices maximum number of slices used
    void autopilot_set_slices_max(uint16_t slices) {
        _autopilot.set_slices_max(T(), slices);
    }
    /// @brief minimum number of slices moved in a scheduler loop
    /// @param slices minimum number of slices used
    void autopilot_set_slices_min(uint16_t slices) {
        _autopilot.set_slices_min(T(), slices);
    }
    /// @brief average number of slices moved in a scheduler loop
    /// @param slices average number of slices used
    void autopilot_set_slices_avg(uint16_t slices) {
        _autopilot.set_slices_avg(T(), slices);
    }
    /// @brief stddev number of slices moved in a scheduler loop
    /// @param slices stddev from average number of slices used
    void autopilot_set_slices_stddev(uint16_t slices) {
        _autopilot.set_slices_stddev(T(), slices);
    }

    /// @brief information from global positioning systems
    /// @return map from gps name to GPS information object
    const std::map<const std::string, GPSInfo*> &gpsinfos() {
        return _gpsinfo;
    }
    /// @brief information from a specific global positioning system
    /// @param name name of GPS unit to return information for
    GPSInfo *gpsinfo(const std::string name) {
        if (_gpsinfo.count(name) == 0) {
            _gpsinfo[name] = new GPSInfo(name);
        }
        return _gpsinfo[name];
    };

    /// @brief magnetometer information
    /// @return map from compass name to compass data
    const std::map<const std::string, Compass*> &compasses() {
        return _compasses;
    }
    /// @brief magnetometer information for specific named compasss
    /// @param name compasss name to retrieve information for
    /// @return compass information for named compass
    Compass *compass(const std::string name) {
        if (_compasses.count(name) == 0) {
            _compasses[name] = new Compass(name);
        }
        return _compasses[name];
    };

    /// @brief IMU information
    /// @return map from IMU name to IMU data
    const std::map<const std::string, IMU*> &imus() {
        return _imus;
    }
    /// @brief IMU information for specific named IMU
    /// @param name IMU to retieve information for
    /// return IMU information for named IMU
    IMU *imu(const std::string name) {
        if (_imus.count(name) == 0) {
            _imus[name] = new IMU(name, _T);
        }
        return _imus[name];
    }

    /// @brief canonical vehicle attitude
    /// @return the vehicle's current canonical attitude
    const Attitude& att() const { return _att; };
    /// @brief canonical vehicle position
    /// @return the vehicle's current canonical position
    const Position& pos() const { return _pos; };
    /// @brief canonical vehicle altitude
    /// @return the vehicle's current canonical absolute altitude
    const Altitude& alt() const { return _alt; };
    /// @brief canonical vehicle velocity
    /// @return the vehicle's current canonical velocity
    const Velocity& vel() const { return _vel; };

    /// @brief canonical vehicle attitude
    /// @return the vehicle's current canonical attitude
    Attitude& att() { return _att; };
    /// @brief canonical vehicle position
    /// @return the vehicle's current canonical position
    Position& pos() { return _pos; };
    /// @brief canonical vehicle altitude
    /// @return the vehicle's current canonical absolute altitude
    Altitude& alt() { return _alt; };
    /// @brief canonical vehicle velocity
    /// @return the vehicle's current canonical velocity
    Velocity& vel() { return _vel; };

    /// @brief vehicle origin position - latitude
    /// @return the vehicle's original position - latitude
    double origin_lat() const { return _origin.lat(); }
    /// @brief modififcation time of the vehicle's original latitude
    /// @return the modification time of the vehicle's original latitude
    uint64_t origin_lat_T() const { return _origin.lat_modtime(); }
    /// @brief vehicle origin position - longitude
    /// @return the vehicle's original position - longitude
    double origin_lon() const { return _origin.lon(); }
    /// @brief modififcation time of the vehicle's original longitude
    /// @return the modification time of the vehicle's original longitude
    uint64_t origin_lon_T() const { return _origin.lon_modtime(); }
    /// @brief set vehicle's original latitude
    /// @param value new vehicle's original latitude
    void set_origin_lat(double value) { _origin.set_lat(T(),value); }
    /// @brief set vehicle's original longitude
    /// @param value new vehicle's original longitude
    void set_origin_lon(double value) { _origin.set_lon(T(),value); }
        
    /// @brief vehicle origin
    /// @return the vehicle's origin
    Position& origin() { return _origin; }

    /// @brief vehicle origin
    /// @return the vehicle's origin
    const Position& origin() const { return _origin; }
    /// @brief vehicle's original altitude
    /// @return the vehicle's original altitude (metres)
    const Altitude &origin_alt() const { return _origin_altitude; }

    /// @brief set vehicle origin altitude
    /// @param value the vehicle's origin altitude (metres)
    void set_origin_altitude(double value) {
        _origin_altitude.set_alt(T(),value);
    }
    /// @brief vehicle's original altitude
    /// @return the vehicle's original altitude (metres)
    double origin_altitude() const { return _origin_altitude.alt(); }
    /// @brief modification time of vehicle's original altitude
    /// @return timestamp when origin altitude was last set (microseconds)
    uint64_t origin_altitude_T() const { return _origin_altitude.alt_modtime(); }

    /// @brief vehicle's altitude relative to origin altitude
    /// @param[out] relative vehicle's altitude relative to its origin
    /// @return true if the relative value could e calculated
    bool relative_alt(double &relative) const;

    /// @brief vehicle's crash state
    /// @return true if the craft is believed to be in a crashed state
    bool crashed() const { return _crashed; }
    /// @brief set the vehicle's crash state
    /// @value the vehicle's new crash state
    void set_crashed(bool value) { _crashed = value; _crashed_T = T(); }

    /// @brief accelerometer clipping state
    /// @return true if any accelerometer is clipping
    bool any_acc_clipping() const;

protected:
    /// @brief navigation state object
    /// @return object representing Navigagion Controller state
    const AV_Nav& nav() const { return _nav; };

    /// @brief current servo outputs (ppm)
    uint16_t _servo_output[9] = { }; // indexed from 1

private:
    /// @brief parameters that have been set
    std::map<const std::string, float> _param;
    /// @brief timestamps parameters were last set
    std::map<const std::string, uint64_t> _param_modtime;
    /// @brief values for parameters if they haven't been seen from a log
    std::map<const std::string, float> _param_defaults = {};
    /// @brief map from sensor name to its health
    std::map<const std::string, bool> _sensors_health = {};

    Attitude _att = { };
    Position _pos = { };
    Altitude _alt = { };
    Velocity _vel = { };
    AV_Nav _nav = { };

    Position _origin = { };
    Altitude _origin_altitude = { };

    EKF _ekf;

    bool _armed = false;

    /// @brief copy state from another vehicle
    void take_state(Base *old);
    
    bool _vehicletype_is_forced = false;
    uint64_t _T = 0;

    uint64_t _time_since_boot;
    uint64_t _time_since_boot_T = 0;

    vehicletype_t _vehicletype = invalid;

    bool _crashed = false; // vehicle's own estimate of whether it has crashed
    uint64_t _crashed_T = 0; // last update time of vehicle's estimate

    Battery _battery;

    AutoPilot _autopilot;

    std::map<const std::string, PositionEstimate*> _position_estimates;
    std::map<const std::string, AttitudeEstimate*> _attitude_estimates;
    std::map<const std::string, AltitudeEstimate*> _altitude_estimates;

    std::map<const std::string, GPSInfo*> _gpsinfo;
    std::map<const std::string, Compass*> _compasses;
    std::map<const std::string, IMU*> _imus;
}; // end class

} // end namespace

#endif
