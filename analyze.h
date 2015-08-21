#ifndef ANALYZE_H
#define ANALYZE_H

#include "mavlink_message_handler.h"
#include "analyzer.h"

class Analyze : public MAVLink_Message_Handler {

public:
    Analyze(int fd, struct sockaddr_in &sa) :
	MAVLink_Message_Handler(fd, sa),
        next_analyzer(0)
        {
        }
    void instantiate_analyzers(INIReader *config);

    void end_of_log();

private:

#define MAX_ANALYZERS 10
    uint8_t next_analyzer;
    Analyzer *analyzer[MAX_ANALYZERS];

    void configure_message_handler(INIReader *config,
                                   MAVLink_Message_Handler *handler,
                                   const char *handler_name);

    void configure_analyzer(INIReader *config,
                            Analyzer *handler,
                            const char *handler_name);

    virtual void handle_decoded_message(uint64_t T, mavlink_ahrs2_t &msg);
    virtual void handle_decoded_message(uint64_t T, mavlink_attitude_t &msg);
    virtual void handle_decoded_message(uint64_t T, mavlink_ekf_status_report_t &msg);
    virtual void handle_decoded_message(uint64_t T, mavlink_heartbeat_t &msg);
    virtual void handle_decoded_message(uint64_t T, mavlink_param_value_t &msg);    
    virtual void handle_decoded_message(uint64_t T, mavlink_servo_output_raw_t &msg);
    virtual void handle_decoded_message(uint64_t T, mavlink_sys_status_t &msg);
    virtual void handle_decoded_message(uint64_t T, mavlink_vfr_hud_t &msg);

    uint32_t results_json(char *buf, const uint32_t buflen);
};

#endif
    
