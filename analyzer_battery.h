#ifndef ANALYZER_BATTERY_H
#define ANALYZER_BATTERY_H

/*
 * analyzer_battery
 *
 */

#include "analyzer.h"

class Analyzer_Battery : public Analyzer {

public:
    Analyzer_Battery(int fd, struct sockaddr_in *sa, AnalyzerVehicle::Base *&vehicle) :
	Analyzer(fd, sa, vehicle)
    {
    }

    const char *name() const { return "Battery"; }
    const char *description() const {
        return "This test will FAIL if the battery level remaining falls below a threshold level";
    }
    bool configure(INIReader *config);
    void handle_decoded_message(uint64_t T, mavlink_sys_status_t &msg) override;
    bool has_failed();

    void results_json_results(Json::Value &root);

private:
    bool seen_sys_status_packets = false;
    double lowest_battery_remaining_seen = 999999999.0f;
    const double low_battery_threshold = 15;
};

#endif


