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
        float roll() { return _roll; };
        float pitch() { return _pitch; };
        float yaw() { return _yaw; };
        void set_roll(uint64_t T, float roll) {
            _roll = roll;
            _roll_modtime = T;
        }
        void set_pitch(uint64_t T, float pitch) {
            _pitch = pitch;
            _pitch_modtime = T;
        }
        void set_yaw(uint64_t T, float yaw) {
            _yaw = yaw;
            _yaw_modtime = T;
        }

    private:
        float _roll;
        float _pitch;
        float _yaw;
        uint64_t _roll_modtime;
        uint64_t _pitch_modtime;
        uint64_t _yaw_modtime;
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

    template <typename packettype>
    class PacketHistory {
    public:
        PacketHistory() :
            next(0) { }
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
    bool _armed;
    std::map<std::string, float> params;

    virtual bool is_flying() = 0;
    virtual bool is_armed() { return _armed; };
    void exceeding_angle_max();

    bool seen_parameter(const char *name);

    virtual void handle_decoded_message(uint64_t T, mavlink_attitude_t&);
    virtual void handle_decoded_message(uint64_t T, mavlink_heartbeat_t&);
    virtual void handle_decoded_message(uint64_t T, mavlink_param_value_t &msg);
    virtual void handle_decoded_message(uint64_t T, mavlink_statustext_t&);
    virtual void handle_decoded_message(uint64_t T, mavlink_servo_output_raw_t &);
    AV_Attitude& att() { return _att; };
    AV_Position& pos() { return _pos; };
    
protected:
    AV_Attitude _att;
    AV_Position _pos;

private:

    PacketHistory<mavlink_heartbeat_t> history_heartbeat;
    PacketHistory<mavlink_servo_output_raw_t> history_servo_output_raw;
    PacketHistory<mavlink_statustext_t> history_statustext;

}; // end class

}; // end namespace

#endif
