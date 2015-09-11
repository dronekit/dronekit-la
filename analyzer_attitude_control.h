#ifndef ANALYZE_ATTITUDE_CONTROL_H
#define ANALYZE_ATTITUDE_CONTROL_H

/*
 * analyze_attitudecontrol
 *
 */

#include "analyzer.h"

#include <set>

class Analyzer_Attitude_Control : public Analyzer {

public:

    Analyzer_Attitude_Control(AnalyzerVehicle::Base *&vehicle) :
	Analyzer(vehicle)
    { }

    const char *name() const { return "Attitude Control"; }
    const char *description() const {
        return "This test will FAIL if the craft's desired attitudes and achieved attitudes do not match for more than a threshold time";
    }

    bool configure(INIReader *config);
    // void handle_decoded_message(uint64_t T, mavlink_nav_controller_output_t &param);
    // void handle_decoded_message(uint64_t T, mavlink_attitude_t &msg);

    void evaluate() override;

private:

    const float offset_warn = 5.0f;
    const float offset_fail = 10.0f;
    const uint32_t duration_min = 250000; // microseconds

    class attitude_control_result : public analyzer_result {
    public:
        uint64_t timestamp_start;
        uint64_t timestamp_stop;
        float deltamax;
        float roll_at_deltamax;
        float desroll_at_deltamax;
        float pitch_at_deltamax;
        float despitch_at_deltamax;
        bool motors_clipping = false;
        std::set<uint8_t> motors_failing;
        std::set<uint8_t> motors_clipping_high;
        std::set<uint8_t> motors_clipping_low;
    };

    void end_of_log(uint32_t packet_count);

    void do_add_severity_score(struct compass_offset_result result);

    #define MAX_ATTITUDE_CONTROL_RESULTS 100
    uint8_t attitude_control_results_offset = 0;
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
