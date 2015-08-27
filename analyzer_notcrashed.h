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
    Analyzer_Crashed(int fd, struct sockaddr_in &sa, AnalyzerVehicle::Base *&vehicle) :
	Analyzer(fd, sa, vehicle),
        seen_packets_attitude(false),
        crashed(false),
        // angle_max(angle_max_default_degrees/180 * M_PI),
        crash_servo_output{ 0 }
    { }

    const uint16_t servo_output_threshold = 1250;
    
    const char *name() { return "Crash Test"; }
    const char *description() {
        return "The vehicle crashed";
    }

    void handle_decoded_message(uint64_t T, mavlink_param_value_t &param) override;
    void handle_decoded_message(uint64_t T, mavlink_attitude_t &att);
    void handle_decoded_message(uint64_t T, mavlink_servo_output_raw_t &servos);
    
private:
    bool has_crashed();
    void evaluate(uint64_t T);

    bool seen_packets_attitude;
    bool crashed;
    float crashed_angle;

    bool exceeded_angle_max;
    uint64_t crashed_timestamp;
    uint64_t exceeded_angle_max_timestamp;

    // const double angle_max_default_degrees = 20.0f;
    
    double crash_servo_output[17]; // FIXME take 17 from somewhere...

    void results_json_results(Json::Value &root);
};

#endif
