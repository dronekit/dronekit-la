#ifndef ANALYZE_ATTITUDE_CONTROL_H
#define ANALYZE_ATTITUDE_CONTROL_H

/*
 * analyze_attitudecontrol
 *
 */

#include "analyzer.h"

class Analyzer_Attitude_Control : public Analyzer {

public:

    Analyzer_Attitude_Control(int fd, struct sockaddr_in &sa, AnalyzerVehicle::Base *&vehicle) :
	Analyzer(fd, sa, vehicle),
        offset_warn(5.0), // degrees
        offset_fail(7.5), // degrees
        attitude_control_results_offset(0)
    { }

    const char *name() { return "Attitude Control"; }
    const char *description() {
        return "Craft's desired attitudes and achieved attitudes match";
    }

    bool configure(INIReader *config);
    void handle_decoded_message(uint64_t T, mavlink_nav_controller_output_t &param);
    void handle_decoded_message(uint64_t T, mavlink_attitude_t &msg);

    bool is_zero(double x) { return x < 0.00001; } // FIXME
    void evaluate(uint64_t T);

private:
    void x_evaluate(uint64_t T);

    const float offset_warn;
    const float offset_fail;

    class attitude_control_result : public analyzer_result {
    public:
        uint64_t timestamp_start;
        uint64_t timestamp_stop;
        float deltamax;
        float roll_at_deltamax;
        float desroll_at_deltamax;
        float pitch_at_deltamax;
        float despitch_at_deltamax;
    };

    void end_of_log(uint32_t packet_count);

    void do_add_evilness(struct compass_offset_result result);

    #define MAX_ATTITUDE_CONTROL_RESULTS 100
    uint8_t attitude_control_results_offset;
    attitude_control_result attitude_control_results[MAX_ATTITUDE_CONTROL_RESULTS];
    bool attitude_control_results_overrun;
    
    void addStatusReason(Json::Value &root, attitude_control_result result);
    uint32_t results_json_attitude_control_status_reason(char *buf, const uint32_t buflen, attitude_control_result result);
    const char * results_json_attitude_control_status_string(attitude_control_result result);
    uint32_t results_json_attitude_control(char *buf, uint32_t buflen);
    void results_json_results(Json::Value &root);

    void results_json_attitude_control(Json::Value &root);
};

#endif
