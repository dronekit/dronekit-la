#ifndef _ANALYZER_VEHICLE
#define  _ANALYZER_VEHICLE

#include <stdint.h>
#include <map>

#include "mavlink/c_library/ardupilotmega/mavlink.h"
#include "mavlink/c_library/common/mavlink.h"

namespace AnalyzerVehicle {

    // AV_Attitude should be the best guess as to what the vehicle's
    // status is - typicall the POS message from dataflash, for
    // example
    class AV_Attitude {
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

    class AV_Position {
    public:
        void set_alt(uint64_t T, float alt) {
            _alt = alt;
            _alt_modtime = T;
        }
        float alt() { return _alt; };
        uint64_t alt_modtime() { return _alt_modtime; };

    private:
        float _lat;
        float _lon;
        float _alt; // relative
        uint64_t _alt_modtime;
    };

    class AV_PosNED {
    public:
        uint16_t N;
        uint16_t E;
        uint16_t D;
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
    Base() :
        _armed(false),
        _att{ },  // not convinced we should be zeroing these -pb2150827
        _pos{ }
        { }

    virtual bool is_flying() = 0;
    virtual bool is_armed() { return _armed; };
    void exceeding_angle_max() const;

    float param(const char *name) { return _param[name]; };
    bool param_seen(const char *name) const;
    uint64_t param_modtime(const char *name) { return _param_modtime[name]; }

    virtual void handle_decoded_message(uint64_t T, mavlink_attitude_t&);
    virtual void handle_decoded_message(uint64_t T, mavlink_heartbeat_t&);
    virtual void handle_decoded_message(uint64_t T, mavlink_nav_controller_output_t &msg);
    virtual void handle_decoded_message(uint64_t T, mavlink_param_value_t &msg);
    virtual void handle_decoded_message(uint64_t T, mavlink_statustext_t&);
    virtual void handle_decoded_message(uint64_t T, mavlink_servo_output_raw_t &);
    AV_Attitude& att() { return _att; };
    AV_Position& pos() { return _pos; };
    AV_Nav& nav() { return _nav; };
    
protected:
    bool _armed;

    std::map<const std::string, float> _param;
    std::map<const std::string, float> _param_modtime;
    AV_Attitude _att;
    AV_Position _pos;
    AV_Nav _nav;

private:

    PacketHistory<mavlink_heartbeat_t> history_heartbeat;
    PacketHistory<mavlink_nav_controller_output_t> history_nav_controller_output;
    PacketHistory<mavlink_servo_output_raw_t> history_servo_output_raw;
    PacketHistory<mavlink_statustext_t> history_statustext;

}; // end class

}; // end namespace

#endif
