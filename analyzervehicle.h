#ifndef _ANALYZER_VEHICLE
#define  _ANALYZER_VEHICLE

#include <stdint.h>
#include <map>

#pragma GCC diagnostic push
#if defined(__GNUC__) && __GNUC__ >= 9
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#endif
#pragma GCC diagnostic ignored "-Wpedantic"
#include "mavlink_c_library/ardupilotmega/mavlink.h"
#include "mavlink_c_library/common/mavlink.h"
#pragma GCC diagnostic pop

#include "analyzer_util.h"
#include "Vector3f.h"

namespace AnalyzerVehicle {

    /// @brief Abstraction of a vehicle attitude.
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

    /// @brief Abstraction of a vehicle attitude rates.
    class AttitudeRate {
    public:
        // all in degrees/second:
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

    /// @brief Abstraction of a vehicle altitude.
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

    /// @brief Altitude according to some sensor or algorithm.
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

    /// @brief Abstraction of a vehicle global position (2D).
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

    /// @brief Abstraction of a vehicle velocity.
    class Velocity {
    public:
        bool is_0d() {
            return ! is_equal(_velocity_scalar, -1.0f);
        }
        bool is_2d() {
            if (is_0d()) {
                return false;
            }
            if (_is_2d_scalar) {
                return true;
            }
            return is_equal(_velocity[2], -1.0f);
        }
        bool is_3d() {
            if (is_0d()) {
                return false;
            }
            if (is_2d()) {
                return false;
            }
            return true;
        }
        double size_2d() {
            return sqrt(_velocity[0]*_velocity[0] + _velocity[1] * _velocity[1]);;
        }
        double size() {
            if (is_0d()) {
                return _velocity_scalar;
            }
            if (is_3d()) {
                // this is the 3D case
                return _velocity.len();
            }
            // this is the 2D case
            return size_2d();
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
        void set_is2D_scalar(const bool value) {
            _is_2d_scalar = value;
        }
        bool is2D_scalar() {
            return _is_2d_scalar;
        }

    private:
        // only one of these two should be set:
        Vector3f _velocity = { };
        double _velocity_scalar = -1;
        bool _have_components = false;
        uint64_t _velocity_modtime = 0;

        bool _is_2d_scalar = false;
    };

    /// @brief State information for a base Extended Kalman Filter.
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

    /// @brief State information for a second-edition Extended Kalman Filter.
    class NKF : public EKF {
    public:
    private:
    };

    /// @brief State information for a third-edition Extended Kalman Filter.
    class XKF : public NKF {
    public:
    private:
    };

    // class AV_PosNED {
    // public:
    //     uint16_t N;
    //     uint16_t E;
    //     uint16_t D;
    // };

    /// @brief Attitude according to some sensor or algorithm.
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

    /// @brief Position according to some sensor or algorithm.
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

    /// @brief Velocity according to some sensor or algorithm.
    class VelocityEstimate {
    public:
        VelocityEstimate(const std::string name) :
            _name(name)
            { }
        VelocityEstimate() :
            _name(NULL),
            _velocity({})
            { }
        const std::string name() { return _name; }
        Velocity &velocity() { return _velocity; }

    private:
        const std::string _name;
        Velocity _velocity = { };
    };

    /// @brief Information on a vehicle battery.
    class Battery {
    public:
        float _remaining = 0; // percent
        uint64_t _remaining_modtime = 0;

        // uint32_t _capacity; // mAh
        // uint64_t _capacity_modtime = 0;

        bool _failsafe_event = 0;
        uint64_t _failsafe_event_T = 0;
    };

    /// @brief Information about the Navigation Controller's output.
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


    /// @brief A source of magnetic field information.
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


    /// @brief A source of GPS information (typically output from a GPS device).
    class GPSInfo {
    public:
        GPSInfo(std::string name) { _name = name; }
        const std::string name() { return _name; }

        uint8_t fix_type() { return _fix_type; }
        void set_fix_type(uint8_t fix_type) { _fix_type = fix_type; }

        double hdop() { return _hdop; }
        void set_hdop(double hdop) { _hdop = hdop; }

        double sacc() { return _sacc; }
        void set_sacc(double sacc) { _sacc = sacc; }

        uint8_t satellites() { return _satellites_visible; }
        void set_satellites(uint8_t satellites) { _satellites_visible = satellites; }

    private:
        std::string _name;

        double _hdop = 0;
        double _sacc = 0;
        uint8_t _satellites_visible = 0;
        uint8_t _fix_type = 0;
    };

    /* may need to factor and subclass this for non-APM-on-PixHawk: */
    /// @brief Information about the AutoPilot.
    class AutoPilot {
    public:
        typedef enum {
            UNKNOWN,
            PX4V2,
        } AutoPilotHardware;
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

        double vcc() { return _vcc; }
        uint64_t vcc_T() { return _vcc_T; }
        void set_vcc(uint64_t T, double);

        void set_hardware(const AutoPilotHardware hw) { _hardware = hw; }
        AutoPilotHardware hardware() const { return _hardware; }

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

        double _vcc;
        uint64_t _vcc_T = 0;

        AutoPilotHardware _hardware = UNKNOWN;
    };

    /// @brief Accelerometer and gyroscope information from onboard IMUs.
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

        /// @brief Retrieve average of gyroscopes by sample count.
        /// @param count number of samples to average over.
        /// @param[out] ret the returned average.
        /// @return True if the average could be found.
        bool gyr_avg(const uint16_t count, Vector3f &ret) const;

        /// @brief Retrieve average of gyroscopes by time
        /// @param T Highest timestamp to use (microseconds, typically _vehicle->T())
        /// @param T_delta Time span over which to take average (microseconds).
        /// @param[out] ret The returned average.
        /// @return True if the average could be found.
        bool gyr_avg(uint64_t T, uint64_t T_delta, Vector3f &ret) const;

        uint64_t acc_T() { // most recent timestamp of all components
            return _acc_T;
        }
        uint64_t gyr_T() { // most recent timestamp of all components
            return _gyr_T;
        }
        void set_acc_T(uint64_t acc_T) { _acc_T = acc_T; }
        void set_gyr(uint64_t T, Vector3f values);

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

        static const uint16_t _gyr_hist_max = 1000;
        struct _timestamped_gyr_hist {
            uint64_t T;
            Vector3f gyr;
        };
        _timestamped_gyr_hist _gyr_hist[_gyr_hist_max] = { };
        uint16_t _gyr_seen = 0;
        uint16_t _gyr_next = 0;

        uint64_t _acc_clip_count_T;
        uint16_t _acc_clip_count = 0;
    };

/// @brief Base class from which all vehicles inherit.
class Base {
public:
    Base() { }
    virtual ~Base() { }

    /// @brief Returns a short string describing the vehicle.
    /// @return A short string describing the vehicle e.g. "Copter".
    virtual const std::string typeString() const { return "Base"; }

    /// @brief Time since autopilot boot (microseconds).
    /// @return Time since autopilot boot (microseconds).
    uint64_t time_since_boot() const {
        return _time_since_boot;
    }
    /// @brief The timestamp at which the "time_since_boot()" timestamp was updated.
    /// @details This timestamp may reflect time on a different system - for example, a MAVLink tlog often uses a Ground Control Station's concept of time.
    /// @return Timestamp that the autopilot boot time was last changed.
    uint64_t time_since_boot_T() const {
        return _time_since_boot_T;
    }
    /// @brief Set the number of microseconds the autopilot has been booted.
    /// @param time_since_boot Microseconds since autopilot boot.
    void set_time_since_boot(const uint64_t time_since_boot) {
        _time_since_boot = time_since_boot;
        _time_since_boot_T = T();
    }
    /// @brief Evaluation of whether the vehicle is flying.
    /// @return ``true`` if it is believed the vehicle is flying.
    virtual bool is_flying() const { return false; }

    /// @brief Vehicle arm status.
    /// @return ``true`` if it believed the vehicle is armed.
    virtual bool is_armed() const { return _armed; };
    /// @brief Set the vehicle arm status.
    /// @param value The new vehicle arm status.
    virtual void set_armed(bool value) {  _armed = value; };

    /// @brief All the different vehicles we have specific tests for.
    enum vehicletype_t {
        invalid = 0,
        copter = 17,
        plane = 19,
        rover = 27,
    };
    /// @brief Vehicle type forced status.
    /// @detail Sometimes the vehicle type can not be determined from a log and must be forced.
    /// @return ``true`` if the vehicle type has been forced.
    bool vehicletype_is_forced() const { return _vehicletype_is_forced; }
    /// @brief Indicates that the vehicle type has been forced.
    /// @param value ``true`` if the vehicle type has been forced.
    void set_vehicletype_is_forced(bool value) { _vehicletype_is_forced = value; }
    /// @brief Vehicle type.
    /// @detail Until the vehicle type is determined, this returns invalid.  After the vehicle type is determined (e.g. from a log), this will return a specific value from the vehicletype_t enumeration indication the vehicle type.
    /// @return The current vehicle type.
    virtual vehicletype_t vehicletype() const {
        return invalid;
    }

    /// @brief Change the vehicle from one type to another.
    /// @detail A base vehicle is used to gather information until we determine what vehicle type we are actually analyzing.  This call is used to switch to a more-specific vehicle type.
    /// @param _vehicle Vehicle to change type of.
    /// @newtype New vehicle type.
    static void switch_vehicletype(Base *&_vehicle, vehicletype_t newtype);

    /// @brief Set a variance value by name.
    /// @param name Name of variance to set (e.g. "pos_horiz").
    /// @param value New value of the variance.
    void ekf_set_variance(const char *name, double value) {
        _ekf.set_variance(T(), name, value);
    }
    /// @brief Variance modification time.
    /// @param name Name of variance to retrieve modification time for.
    /// @return timestamp Timestamp at which variance was last changed (microseconds).
    uint64_t ekf_variance_T(std::string name) const {
        return _ekf.variance_T(name);
    }
    /// @brief Set EKF status flags.
    /// @param flags EKF self-assessment status flags.
    void ekf_set_flags(uint16_t flags) {
        _ekf.set_flags(T(), flags);
    }
    /// @brief EKF status flags.
    /// @return EKF self-assessment status flags.
    uint16_t ekf_flags() const {
        return _ekf.flags();
    }

    /// @brief EKF status flags modification time.
    /// @return timestamp Timestamp at which the EKF self-assessment flags were last changed (microseconds).
    uint64_t ekf_flags_T() const {
        return _ekf.flags_T();
    }
    /// @brief All EKF variances.
    /// @return Returns all EKF variances as a map from name to value.
    std::map<const std::string, double> ekf_variances() {
        return _ekf.variances();
    }

    /// @brief Set a variance value by name.
    /// @param name Name of variance to set (e.g. "pos_horiz").
    /// @param value New value of the variance.
    void nkf_set_variance(const char *name, double value) {
        _nkf.set_variance(T(), name, value);
    }
    uint64_t nkf_variance_T(std::string name) const {
        return _nkf.variance_T(name);
    }
    /// @brief Set NKF (EKFv2) status flags.
    /// @param flags NKF (EKFv2 self-assessment status flags.
    void nkf_set_flags(uint16_t flags) {
        _nkf.set_flags(T(), flags);
    }
    /// @brief NKF status flags.
    /// @return NKF self-assessment status flags.
    uint16_t nkf_flags() const {
        return _nkf.flags();
    }

    /// @brief NKF (EKF2) status flags modification time.
    /// @return timestamp Timestamp at which the NKF self-assessment flags were last changed (microseconds).
    uint64_t nkf_flags_T() const {
        return _nkf.flags_T();
    }
    /// @brief All NKF (EKF2) variances.
    /// @return Returns all NKF (EKF2) variances as a map from name to value.
    std::map<const std::string, double> nkf_variances() {
        return _nkf.variances();
    }


    /// @brief Set a variance value by name.
    /// @param name Name of variance to set (e.g. "pos_horiz").
    /// @param value New value of the variance.
    void xkf_set_variance(const char *name, double value) {
        _xkf.set_variance(T(), name, value);
    }
    uint64_t xkf_variance_T(std::string name) const {
        return _xkf.variance_T(name);
    }
    /// @brief Set XKF (EKF3) status flags.
    /// @param flags XKF (EKF3 self-assessment status flags.
    void xkf_set_flags(uint16_t flags) {
        _xkf.set_flags(T(), flags);
    }
    /// @brief XKF status flags.
    /// @return XKF self-assessment status flags.
    uint16_t xkf_flags() const {
        return _xkf.flags();
    }

    /// @brief XKF (EKF3) status flags modification time.
    /// @return timestamp Timestamp at which the XKF self-assessment flags were last changed (microseconds).
    uint64_t xkf_flags_T() const {
        return _xkf.flags_T();
    }
    /// @brief All XKF (EKF3) variances.
    /// @return Returns all XKF (EKF3) variances as a map from name to value.
    std::map<const std::string, double> xkf_variances() {
        return _xkf.variances();
    }

    /// @brief Retrieve parameter value.
    /// @param name Parameter value to retrieve.
    /// @detail This function will abort if the parameter has not been seen and no default is known.
    /// @return Parameter value from log or defaults.
    float param(const std::string name) const;
    /// @brief Retrieve parameter value.
    /// @detail Returns parameter value ONLY from input, not from defaults.
    /// @param name Parameter value to retrieve.
    /// @param[out] ret Parameter value.
    /// @return ``true`` if the parameter has been seen in the log.
    bool param(const char *name, float &ret) const;
    /// @brief Retrieve parameter value.
    /// @detail Returns parameter value ONLY from input, not from defaults.
    /// @param name Parameter value to retrieve.
    /// @param[out] ret Parameter value.
    /// @return ``true`` if the parameter has been seen in the log.
    bool param(const std::string name, float &ret) const;
    /// @brief Number of parameters seen.
    /// @return Number of parameters seen in input.
    uint16_t param_count() const { return _param.size(); };
    /// @brief Parameter status.
    /// @return ``true`` if the parameter has been seen in the log.
    bool param_seen(const std::string name) const;
    /// @brief Timestamp at which a parameter was last modified (microseconds).
    /// @param name Parameter modification time to retrieve.
    /// @return Parameter modification time (microseconds).
    uint64_t param_T(const std::string name) const;
    /// @brief Set a parameter.
    /// @param name Parameter to set.
    /// @param value New value of parameter.
    void param_set(const char *name, const float value);
    /// @brief Return the default value for a parameter.
    /// @detail Default parameters are supplied so that some analysis can be done even in the absence of parameters in the logs.
    /// @param name Parameter to retrieve default value for.
    /// @param[out] ret Value of parameter.
    /// @return ``true`` if a default was found for the parameters.
    virtual bool param_default(const char *name, float &ret) const;
    /// @brief Retrieve a parameter value.
    /// @detail Returns the value last set for this parameter, or a default if it has not been set yet.
    /// @param name Parameter to retrieve default value for.
    /// @param[out] ret Value of parameter.
    /// @return ``true`` if the parameter value was found.
    bool param_with_defaults(const char *name, float &ret) const;
    /// @brief Retrieves a parameter value and abort()s if it not found.
    /// @detail Returns the value last set for this parameter, or a default if it has not been set yet.
    /// @param name Parameter to retrieve default value for.
    /// @return Value of parameter.
    float require_param_with_defaults(const char *name) const;

    /// @brief Set outputs for all servos.
    /// @param ch1 current output for servo 1 (ppm).
    /// @param ch2 current output for servo 2 (ppm).
    /// @param ch3 current output for servo 3 (ppm).
    /// @param ch4 current output for servo 4 (ppm).
    /// @param ch5 current output for servo 5 (ppm).
    /// @param ch6 current output for servo 6 (ppm).
    /// @param ch7 current output for servo 7 (ppm).
    /// @param ch8 current output for servo 8 (ppm).
    void set_servo_output(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4, uint16_t ch5, uint16_t ch6, uint16_t ch7, uint16_t ch8);
    /// @brief Set outputs for single servo.
    /// @param channel_number Servo to set value for.
    /// @param value New value for servo output (ppm).
    void set_servo_output(uint8_t channel_number, uint16_t value);

    /// @brief Return current output for a specific servo.
    /// @param channel_number Servo number.
    /// @return Current servo output (ppm).
    uint16_t servo_output(uint8_t channel_number) const {
        return _servo_output[channel_number];
    }

    /// @brief Set timestamp distinguishing logging events.
    /// @detail This may or may not be a real wall-clock time.
    /// @param time_us New timestamp (microseconds).
    void set_T(const uint64_t time_us);
    /// @brief Current timestamp.
    /// @return Current universe timestamp.
    uint64_t T() { return _T; }

    /// @brief Canonical vehicle attitude - roll.
    /// @return Canonical attitude - roll (degrees).
    float roll() const { return att().roll(); }
    /// @brief Canonical vehicle attitude - pitch (degrees).
    /// @return Canonical attitude - pitch.
    float pitch() const { return att().pitch(); }
    /// @brief Canonical vehicle attitude - yaw.
    /// @return Canonical attitude - yaw (degrees).
    float yaw() const { return att().yaw(); }

    /// @brief Modification time - roll.
    /// @return Timestamp roll was last modified (microseconds).
    uint64_t roll_modtime() const { return att().roll_modtime(); }
    /// @brief Modification time - pitch.
    /// @return Timestamp pitch was last modified (microseconds).
    uint64_t pitch_modtime() const { return att().pitch_modtime(); }
    /// @brief Modification time - yaw.
    /// @return Timestamp yaw was last modified (microseconds).
    uint64_t yaw_modtime() const { return att().yaw_modtime(); }

    /// @brief Set canonical vehicle atttiude - roll.
    /// @param new Canonical attitude - roll (degrees).
    void set_roll(float roll) { att().set_roll(T(), roll); }
    /// @brief Set canonical vehicle atttiude - pitch.
    /// @param new Canonical attitude - pitch (degrees).
    void set_pitch(float roll) { att().set_pitch(T(), roll); }
    /// @brief Set canonical vehicle atttiude - yaw.
    /// @param new Canonical attitude - yaw (degrees).
    void set_yaw(float roll) { att().set_yaw(T(), roll); }

    /// @brief set Navigation Controller desired attitude - roll.
    /// @param roll New desired attitude - roll.
    void set_desroll(float roll) { _nav.set_desroll(T(), roll); };
    /// @brief Set Navigation Controller desired attitude - pitch.
    /// @param roll New desired attitude - pitch.
    void set_despitch(float pitch) { _nav.set_despitch(T(), pitch); };
    /// @brief Set Navigation Controller desired attitude - yaw.
    /// @param roll New desired attitude - yaw.
    void set_desyaw(float yaw) { _nav.set_desyaw(T(), yaw); };

    /// @brief Navigation Controller desired attitude - roll.
    float desroll() const { return nav().desroll(); }
    /// @brief Navigation Controller desired attitude - pitch.
    float despitch() const { return nav().despitch(); }
    /// @brief Navigation Controller desired attitude - yaw.
    float desyaw() const { return nav().desyaw(); }

    /// @brief Set canonical vehicle altitude.
    /// @param value New vehicle altitude.
    void set_altitude(float value) {
        alt().set_alt(T(), value);
    }
    /// @brief Canonical vehicle altitude.
    /// @return The vehicle's canonical altitude.
    float altitude() const { return alt().alt(); };
    /// @brief Canonical vehicle altitude modification time.
    /// @return Timestamp The canonical vehicle altitude was modified (microseconds).
    uint64_t alt_modtime() const { return alt().alt_modtime(); }

    /// @brief Set vehicle canonical latitude.
    /// @param lat New vehicle canonical latitude.
    void set_lat(double lat) { pos().set_lat(T(), lat); }
    /// @brief Set vehicle canonical longitude.
    /// @param lat New vehicle canonical latitude.
    void set_lon(double lon) { pos().set_lon(T(), lon); }
    // @brief Vehicle canonical latitude.
    // @return Vehicle's canonical latitude.
    double lat() { return pos().lat(); }
    // @brief Vehicle canonical longitude.
    // @return Vehicle's canonical longitude.
    double lon() { return pos().lon(); }

    /// @brief Retrieve named position estimate.
    /// @param name Name of position estimate to retrieve.
    /// @return A position estimate.
    PositionEstimate *position_estimate(const std::string name) {
        if (_position_estimates.count(name) == 0) {
            _position_estimates[name] = new PositionEstimate(name);
        }
        return _position_estimates[name];
    };

    /// @brief Retrieve named attitude estimate.
    /// @param name Name of attitude estimate to retrieve.
    /// @return An attitude estimate.
    AttitudeEstimate *attitude_estimate(const std::string name) {
        if (_attitude_estimates.count(name) == 0) {
            _attitude_estimates[name] = new AttitudeEstimate(name);
        }
        return _attitude_estimates[name];
    };

    /// @brief Retrieve named altitude estimate.
    /// @param name Name of altitude estimate to retrieve.
    /// @return An altitude estimate.
    AltitudeEstimate *altitude_estimate(const std::string name) {
        if (_altitude_estimates.count(name) == 0) {
            _altitude_estimates[name] = new AltitudeEstimate(name);
        }
        return _altitude_estimates[name];
    };

    /// @brief Retrieve named velocity estimate.
    /// @param name Name of velocity estimate to retrieve.
    /// @return A velocity estimate.
    VelocityEstimate *velocity_estimate(const std::string name) {
        if (_velocity_estimates.count(name) == 0) {
            _velocity_estimates[name] = new VelocityEstimate(name);
        }
        return _velocity_estimates[name];
    };

    /// @brief Distance from canonical vehicle position to canonical origin.
    /// @return Distance from origin, or -1 if we just don't know.
    double distance_from_origin();

    /// @brief Hardware diagnostics.
    /// @return Map of sensor name to its boolean health.
    std::map<const std::string, bool> sensors_health() {
        return _sensors_health;
    }
    /// @brief Set the health status of a sensor by name.
    /// @param name Name of sensor.
    /// @param value New health value of sensor.
    void sensor_set_healthy(std::string name, bool value) {
        _sensors_health[name] = value;
    }

    /// @brief Subsystem health
    /// @return Map of subsystem ID to its status
    std::map<uint32_t, uint32_t> subsys_errors() {
        return _subsys_error;
    }
    /// @brief Set an error flag value
    /// @param subsys subsystem ID
    /// @param code error code
    void set_subsys_error_code(uint32_t subsys, uint32_t code) {
        _subsys_error[subsys] = code;
    }

    /// @brief Indicate amount of flight battery remaining.
    /// @param percent Percentage of charge remaining.
    void set_battery_remaining(float percent) {
        _battery._remaining = percent;
        _battery._remaining_modtime = T();
    }
    /// @brief Amount of battery remaining.
    /// @return Amount of battery remaining (percentage).
    float battery_remaining() {
        return _battery._remaining;
    }
    /// @brief Battery remaining modification time.
    /// @return Timestamp when battery remaining was last modified (microseconds).
    uint64_t battery_remaining_T() {
        return _battery._remaining_modtime;
    }
    /// @brief Indicate a battery is in failsafe.
    /// @param in_failsafe ``true`` if battery is in failsafe.
    void set_battery_in_failsafe(bool in_failsafe) {
        _battery._failsafe_event = in_failsafe;
        _battery._failsafe_event_T = T();
    }
    /// @brief Battery failsafe status.
    /// @return ``true`` if the battery is in failsafe.
    bool battery_in_failsafe() {
        return _battery._failsafe_event;
    }
    /// @brief Battery in failsafe modification time.
    /// @return Timestamp when battery failsafe was last modified (microseconds).
    uint64_t battery_in_failsafe_T() {
        return _battery._failsafe_event_T;
    }

    /// @brief Attitude estimates.
    /// @return Map of attitude estimate name to attitude estimate.
    const std::map<const std::string, AttitudeEstimate*> &attitude_estimates() {
        return _attitude_estimates;
    }
    /// @brief Altitude estimates.
    /// @return Map of altitude estimate name to altitude estimate.
    const std::map<const std::string, AltitudeEstimate*> &altitude_estimates() {
        return _altitude_estimates;
    }
    /// @brief Position estimates.
    /// @return Map of position estimate name to position estimate.
    const std::map<const std::string, PositionEstimate*> &position_estimates() {
        return _position_estimates;
    }
    /// @brief Velocity estimates.
    /// @return Map of velocity estimate name to velocity estimate.
    const std::map<const std::string, VelocityEstimate*> &velocity_estimates() {
        return _velocity_estimates;
    }


    /// @brief Autopilot status information.
    /// @return Object containing autopilot status information.
    AutoPilot &autopilot() {
        return _autopilot;
    }

    /// @brief Set number of autopilot scheduler overruns.
    /// @param scheduler Overruns in last loopcount() loops.
    void autopilot_set_overruns(uint16_t overruns) {
        _autopilot.set_overruns(T(), overruns);
    }
    /// @brief Set number of loops for scheduler data.
    /// @param count Number of loops for scheduler data.
    void autopilot_set_loopcount(uint16_t count) {
        _autopilot.set_loopcount(T(), count);
    }
    /// @brief Maximum number of slices moved in a scheduler loop.
    /// @param slices Maximum number of slices used.
    void autopilot_set_slices_max(uint16_t slices) {
        _autopilot.set_slices_max(T(), slices);
    }
    /// @brief Minimum number of slices moved in a scheduler loop.
    /// @param slices Minimum number of slices used.
    void autopilot_set_slices_min(uint16_t slices) {
        _autopilot.set_slices_min(T(), slices);
    }
    /// @brief Average number of slices moved in a scheduler loop.
    /// @param slices Average number of slices used.
    void autopilot_set_slices_avg(uint16_t slices) {
        _autopilot.set_slices_avg(T(), slices);
    }
    /// @brief Stddev number of slices moved in a scheduler loop.
    /// @param slices Stddev from average number of slices used.
    void autopilot_set_slices_stddev(uint16_t slices) {
        _autopilot.set_slices_stddev(T(), slices);
    }

    /// @brief Set vcc of autopilot
    /// @param vcc vcc supplied to autopilot
    void autopilot_set_vcc(double slices) {
        _autopilot.set_vcc(T(), slices);
    }


    /// @brief Information from global positioning systems.
    /// @return Map from GPS name to GPS information object.
    const std::map<const std::string, GPSInfo*> &gpsinfos() {
        return _gpsinfo;
    }
    /// @brief Information from a specific global positioning system.
    /// @param name Name of GPS unit to return information for.
    GPSInfo *gpsinfo(const std::string name) {
        if (_gpsinfo.count(name) == 0) {
            _gpsinfo[name] = new GPSInfo(name);
        }
        return _gpsinfo[name];
    };

    /// @brief Magnetometer information.
    /// @return Map from compass name to compass data.
    const std::map<const std::string, Compass*> &compasses() {
        return _compasses;
    }
    /// @brief Magnetometer information for specific named compass.
    /// @param name Compass name to retrieve information for.
    /// @return Compass information for named compass.
    Compass *compass(const std::string name) {
        if (_compasses.count(name) == 0) {
            _compasses[name] = new Compass(name);
        }
        return _compasses[name];
    };

    /// @brief IMU information.
    /// @return Map from IMU name to IMU data.
    const std::map<const std::string, IMU*> &imus() {
        return _imus;
    }
    /// @brief IMU information for specific named IMU.
    /// @param name IMU to retrieve information for.
    /// return IMU information for named IMU.
    IMU *imu(const std::string name) {
        if (_imus.count(name) == 0) {
            _imus[name] = new IMU(name, _T);
        }
        return _imus[name];
    }

    /// @brief Canonical vehicle attitude.
    /// @return The vehicle's current canonical attitude.
    const Attitude& att() const { return _att; };
    /// @brief Canonical vehicle attitude rates.
    /// @return The vehicle's current canonical attitude rates.
    const AttitudeRate& rate() const { return _rate; };
    /// @brief Canonical vehicle position.
    /// @return The vehicle's current canonical position.
    const Position& pos() const { return _pos; };
    /// @brief Canonical vehicle altitude.
    /// @return The vehicle's current canonical absolute altitude.
    const Altitude& alt() const { return _alt; };
    /// @brief Canonical vehicle velocity.
    /// @return The vehicle's current canonical velocity.
    const Velocity& vel() const { return _vel; };

    /// @brief Canonical vehicle attitude.
    /// @return The vehicle's current canonical attitude.
    Attitude& att() { return _att; };
    /// @brief Canonical vehicle attitude rates.
    /// @return The vehicle's current canonical attitude rates.
    AttitudeRate& rate() { return _rate; };
    /// @brief Canonical vehicle position.
    /// @return The vehicle's current canonical position.
    Position& pos() { return _pos; };
    /// @brief Canonical vehicle altitude.
    /// @return The vehicle's current canonical absolute altitude.
    Altitude& alt() { return _alt; };
    /// @brief Canonical vehicle velocity.
    /// @return The vehicle's current canonical velocity.
    Velocity& vel() { return _vel; };

    /// @brief Vehicle origin position - latitude.
    /// @return The vehicle's original position - latitude.
    double origin_lat() const { return _origin.lat(); }
    /// @brief Modification time of the vehicle's original latitude.
    /// @return The modification time of the vehicle's original latitude.
    uint64_t origin_lat_T() const { return _origin.lat_modtime(); }
    /// @brief Vehicle origin position - longitude.
    /// @return The vehicle's original position - longitude.
    double origin_lon() const { return _origin.lon(); }
    /// @brief Modification time of the vehicle's original longitude.
    /// @return The modification time of the vehicle's original longitude.
    uint64_t origin_lon_T() const { return _origin.lon_modtime(); }
    /// @brief Set vehicle's original latitude.
    /// @param value New vehicle's original latitude.
    void set_origin_lat(double value) { _origin.set_lat(T(),value); }
    /// @brief Set vehicle's original longitude.
    /// @param value New vehicle's original longitude.
    void set_origin_lon(double value) { _origin.set_lon(T(),value); }

    /// @brief Vehicle origin.
    /// @return The vehicle's origin.
    Position& origin() { return _origin; }

    /// @brief Vehicle origin.
    /// @return The vehicle's origin.
    const Position& origin() const { return _origin; }
    /// @brief Vehicle's original altitude.
    /// @return The vehicle's original altitude (metres).
    const Altitude &origin_alt() const { return _origin_altitude; }

    /// @brief Set vehicle origin altitude.
    /// @param value The vehicle's origin altitude (metres).
    void set_origin_altitude(double value) {
        _origin_altitude.set_alt(T(),value);
    }
    /// @brief Vehicle's original altitude.
    /// @return The vehicle's original altitude (metres).
    double origin_altitude() const { return _origin_altitude.alt(); }
    /// @brief Modification time of vehicle's original altitude.
    /// @return Timestamp when origin altitude was last set (microseconds).
    uint64_t origin_altitude_T() const { return _origin_altitude.alt_modtime(); }

    /// @brief vehicle's altitude relative to origin altitude.
    /// @param[out] relative Vehicle's altitude relative to its origin.
    /// @return true if the relative value could e calculated.
    bool relative_alt(double &relative) const;

    /// @brief Vehicle's crash state.
    /// @return ``true`` if the craft is believed to be in a crashed state.
    bool crashed() const { return _crashed; }
    /// @brief Set the vehicle's crash state.
    /// @value The vehicle's new crash state.
    void set_crashed(bool value) { _crashed = value; _crashed_T = T(); }

    /// @brief Accelerometer clipping state.
    /// @return ``true`` if any accelerometer is clipping.
    bool any_acc_clipping() const;

protected:
    /// @brief Navigation state object.
    /// @return Object representing Navigation Controller state.
    const AV_Nav& nav() const { return _nav; };

    /// @brief Current servo outputs (ppm).
    uint16_t _servo_output[9] = { }; // indexed from 1

private:
    /// @brief Parameters that have been set.
    std::map<const std::string, float> _param;
    /// @brief Timestamps parameters were last set.
    std::map<const std::string, uint64_t> _param_T;
    /// @brief Values for parameters if they haven't been seen from a log.
    std::map<const std::string, float> _param_defaults = {
        { "AHRS_EKF_TYPE", 1.0 }
    };
    /// @brief Map from sensor name to its health.
    std::map<const std::string, bool> _sensors_health = {};

    /// @brief Map from subsystem to its current error status
    std::map<uint32_t, uint32_t> _subsys_error = {};

    /// @brief Map from subsystem id to its name
    /// @note Correct only for ArduCopter ATM
    /// @note these defines taken from ArduCopter/defines.h
#define ERROR_SUBSYSTEM_MAIN                1
#define ERROR_SUBSYSTEM_RADIO               2
#define ERROR_SUBSYSTEM_COMPASS             3
#define ERROR_SUBSYSTEM_OPTFLOW             4
#define ERROR_SUBSYSTEM_FAILSAFE_RADIO      5
#define ERROR_SUBSYSTEM_FAILSAFE_BATT       6
#define ERROR_SUBSYSTEM_FAILSAFE_GPS        7   // not used
#define ERROR_SUBSYSTEM_FAILSAFE_GCS        8
#define ERROR_SUBSYSTEM_FAILSAFE_FENCE      9
#define ERROR_SUBSYSTEM_FLIGHT_MODE         10
#define ERROR_SUBSYSTEM_GPS                 11
#define ERROR_SUBSYSTEM_CRASH_CHECK         12
#define ERROR_SUBSYSTEM_FLIP                13
#define ERROR_SUBSYSTEM_AUTOTUNE            14
#define ERROR_SUBSYSTEM_PARACHUTE           15
#define ERROR_SUBSYSTEM_EKFCHECK            16
#define ERROR_SUBSYSTEM_FAILSAFE_EKFINAV    17
#define ERROR_SUBSYSTEM_BARO                18
#define ERROR_SUBSYSTEM_CPU                 19
#define ERROR_SUBSYSTEM_FAILSAFE_ADSB       20
#define ERROR_SUBSYSTEM_TERRAIN             21
#define ERROR_SUBSYSTEM_NAVIGATION          22
#define ERROR_SUBSYSTEM_FAILSAFE_TERRAIN    23
#define ERROR_SUBSYSTEM_EKF_PRIMARY         24

// general error codes
#define ERROR_CODE_ERROR_RESOLVED           0
#define ERROR_CODE_FAILED_TO_INITIALISE     1
#define ERROR_CODE_UNHEALTHY                4
// subsystem specific error codes -- radio
#define ERROR_CODE_RADIO_LATE_FRAME         2
// subsystem specific error codes -- failsafe_thr, batt, gps
#define ERROR_CODE_FAILSAFE_RESOLVED        0
#define ERROR_CODE_FAILSAFE_OCCURRED        1
// subsystem specific error codes -- compass
#define ERROR_CODE_COMPASS_FAILED_TO_READ   2
// subsystem specific error codes -- main
#define ERROR_CODE_MAIN_INS_DELAY           1
// subsystem specific error codes -- crash checker
#define ERROR_CODE_CRASH_CHECK_CRASH        1
#define ERROR_CODE_CRASH_CHECK_LOSS_OF_CONTROL 2
// subsystem specific error codes -- flip
#define ERROR_CODE_FLIP_ABANDONED           2
// subsystem specific error codes -- terrain
#define ERROR_CODE_MISSING_TERRAIN_DATA     2
// subsystem specific error codes -- navigation
#define ERROR_CODE_FAILED_TO_SET_DESTINATION    2
#define ERROR_CODE_RESTARTED_RTL            3
#define ERROR_CODE_FAILED_CIRCLE_INIT       4
#define ERROR_CODE_DEST_OUTSIDE_FENCE       5

// parachute failed to deploy because of low altitude or landed
#define ERROR_CODE_PARACHUTE_TOO_LOW        2
#define ERROR_CODE_PARACHUTE_LANDED         3
// EKF check definitions
#define ERROR_CODE_EKFCHECK_BAD_VARIANCE       2
#define ERROR_CODE_EKFCHECK_VARIANCE_CLEARED   0
// Baro specific error codes
#define ERROR_CODE_BARO_GLITCH              2
// GPS specific error coces
#define ERROR_CODE_GPS_GLITCH               2

    Attitude _att = { };
    AttitudeRate _rate = { };
    Position _pos = { };
    Altitude _alt = { };
    Velocity _vel = { };
    AV_Nav _nav = { };

    Position _origin = { };
    Altitude _origin_altitude = { };

    EKF _ekf;
    NKF _nkf;
    XKF _xkf;

    bool _armed = false;

    /// @brief Copy state from another vehicle.
    void take_state(Base *old);

    bool _vehicletype_is_forced = false;
    uint64_t _T = 0;

    uint64_t _time_since_boot;
    uint64_t _time_since_boot_T = 0;

    bool _crashed = false; // vehicle's own estimate of whether it has crashed.
    uint64_t _crashed_T = 0; // last update time of vehicle's estimate.

    Battery _battery;

    AutoPilot _autopilot;

    std::map<const std::string, PositionEstimate*> _position_estimates;
    std::map<const std::string, AttitudeEstimate*> _attitude_estimates;
    std::map<const std::string, AltitudeEstimate*> _altitude_estimates;
    std::map<const std::string, VelocityEstimate*> _velocity_estimates;

    std::map<const std::string, GPSInfo*> _gpsinfo;
    std::map<const std::string, Compass*> _compasses;
    std::map<const std::string, IMU*> _imus;
}; // end class

} // end namespace

#endif
