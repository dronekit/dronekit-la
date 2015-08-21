#ifndef ANALYZE_CRASHED_H
#define ANALYZE_CRASHED_H

/*
 * analyze_crashed
 *
 */

#include "analyzer.h"

#define angle_max_default_degrees 20

class Analyzer_Crashed : public Analyzer {

public:
    Analyzer_Crashed(int fd, struct sockaddr_in &sa) :
	Analyzer(fd, sa),
        seen_packets_attitude(false),
        crashed(false),
        angle_max(angle_max_default_degrees/180 * M_PI),
        angle_max_achieved(0),
        last_servo_output{ 0 }
    { }

    const uint16_t servo_output_threshold = 1250;
    
    const char *name() { return "Crash Test"; }
    const char *description() {
        return "The vehicle crashed";
    }

    bool configure(INIReader *config);
    void handle_decoded_message(uint64_t T, mavlink_param_value_t &param) override;
    void handle_decoded_message(uint64_t T, mavlink_attitude_t &att);
    void handle_decoded_message(uint64_t T, mavlink_servo_output_raw_t &servos);
    
    bool is_zero(double x) { return x < 0.00001; } // FIXME

private:
    bool has_crashed();
    void evaluate_crashed(uint64_t T);

    bool seen_packets_attitude;
    bool crashed;
    double angle_max; // radians
    bool exceeded_angle_max;
    bool angle_max_achieved;
    uint64_t crashed_timestamp;
    uint64_t exceeded_angle_max_timestamp;

    // const double angle_max_default_degrees = 20.0f;
    
    double last_servo_output[17]; // FIXME take 17 from somewhere...

    void results_json_results(Json::Value &root);
};

#endif
