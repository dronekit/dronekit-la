#ifndef ANALYZER_BROWNOUT_H
#define ANALYZER_BROWNOUT_H

/*
 * analyzer_brownout
 *
 */

#include "analyzer.h"

class Analyzer_Brownout : public Analyzer {

public:
    Analyzer_Brownout(int fd, struct sockaddr_in &sa) :
	Analyzer(fd, sa),
        seen_packets(false),
        last_altitude(0.0f),
        last_servo_output{ 0 }
    {
    }

    const char *name() { return "Brownout"; }
    const char *description() {
        return "Log does not end while vehicle appears to be flying";
    }
    bool configure(INIReader *config);
    void handle_decoded_message(uint64_t T, mavlink_vfr_hud_t &) override;
    void handle_decoded_message(uint64_t T, mavlink_servo_output_raw_t &) override;
    bool has_failed();

    void results_json_results(Json::Value &root);

private:
    bool seen_packets;
    double last_altitude;

    double last_servo_output[17];

    const double max_last_altitude = 5.0f; // metres
};

#endif


