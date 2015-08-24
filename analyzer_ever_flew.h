#ifndef ANALYZE_EVER_FLEW_H
#define ANALYZE_EVER_FLEW_H

/*
 * analyze_ever_flew
 *
 */

#include "analyzer.h"

class Analyzer_Ever_Flew : public Analyzer {
public:
    Analyzer_Ever_Flew(int fd, struct sockaddr_in &sa) :
	Analyzer(fd, sa),
        ever_armed(false),
        servos_past_threshold(false),
        pass_timestamp(0)
    { }

    const char *name() { return "Ever Flew"; }
const char *description() { return "The vehicle flew, as defined by having ever armed and having the motor outputs pass a threshold value"; }

    bool configure(INIReader *config);
    void handle_decoded_message(uint64_t T, mavlink_heartbeat_t &hearbeat) override;
    void handle_decoded_message(uint64_t T, mavlink_servo_output_raw_t &servos) override;

    void results_json_results(Json::Value&);
private:
    bool ever_armed;
    bool servos_past_threshold;
    uint64_t pass_timestamp;

    bool has_passed();
};

#endif
