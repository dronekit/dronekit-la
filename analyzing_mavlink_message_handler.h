#ifndef _ANALYZING_MAVLINK_MESSAGE_HANDLER_H
#define _ANALYZING_MAVLINK_MESSAGE_HANDLER_H

#include "mavlink_message_handler.h"
#include "analyze.h"

class Analyzing_MAVLink_Message_Handler : public MAVLink_Message_Handler {
public:
    Analyzing_MAVLink_Message_Handler(Analyze *analyze,
                                      AnalyzerVehicle::Base *&vehicle) :
        MAVLink_Message_Handler(),
        _analyze(analyze),
        _vehicle(vehicle) { }

private:
    virtual void handle_decoded_message(uint64_t T, mavlink_ahrs2_t &msg);
    virtual void handle_decoded_message(uint64_t T, mavlink_attitude_t &msg);
    virtual void handle_decoded_message(uint64_t T, mavlink_ekf_status_report_t &msg);
    virtual void handle_decoded_message(uint64_t T, mavlink_heartbeat_t &msg);
    virtual void handle_decoded_message(uint64_t T, mavlink_nav_controller_output_t &msg);
    virtual void handle_decoded_message(uint64_t T, mavlink_param_value_t &msg);    
    virtual void handle_decoded_message(uint64_t T, mavlink_servo_output_raw_t &msg);
    virtual void handle_decoded_message(uint64_t T, mavlink_statustext_t &msg);
    virtual void handle_decoded_message(uint64_t T, mavlink_sys_status_t &msg);
    virtual void handle_decoded_message(uint64_t T, mavlink_vfr_hud_t &msg);

    void end_of_log(uint32_t packet_count) override;
    
    Analyze *_analyze;
    AnalyzerVehicle::Base *&_vehicle;

    void set_vehicle_copter();
};


#endif
