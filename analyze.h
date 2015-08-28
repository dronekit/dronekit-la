#ifndef ANALYZE_H
#define ANALYZE_H

#include "mavlink_message_handler.h"
#include "analyzer.h"
#include "analyzervehicle_copter.h"

#include "analyzer_util.h"

class Analyze : public MAVLink_Message_Handler {

public:
    Analyze(int fd, struct sockaddr_in *sa) :
	MAVLink_Message_Handler(fd, sa),
        vehicle(NULL),
        _output_style(OUTPUT_JSON),
        next_analyzer(0)
        {
            start_time = now();
        }
    void instantiate_analyzers(INIReader *config);

    void end_of_log(uint32_t packet_count);

    enum output_style_option {
        OUTPUT_JSON = 17,
        OUTPUT_PLAINTEXT,
        OUTPUT_HTML,
    };

    void set_output_style(output_style_option option) { _output_style = option;}

private:
    uint64_t start_time;

    AnalyzerVehicle::Base *vehicle;

    output_style_option _output_style;
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
    virtual void handle_decoded_message(uint64_t T, mavlink_nav_controller_output_t &msg);
    virtual void handle_decoded_message(uint64_t T, mavlink_param_value_t &msg);    
    virtual void handle_decoded_message(uint64_t T, mavlink_servo_output_raw_t &msg);
    virtual void handle_decoded_message(uint64_t T, mavlink_statustext_t &msg);
    virtual void handle_decoded_message(uint64_t T, mavlink_sys_status_t &msg);
    virtual void handle_decoded_message(uint64_t T, mavlink_vfr_hud_t &msg);

    void results_json(Json::Value &root);

    void output_plaintext(Json::Value &root);
};

#endif
    
