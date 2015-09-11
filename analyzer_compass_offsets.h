#ifndef ANALYZE_COMPASS_OFFSETS_H
#define ANALYZE_COMPASS_OFFSETS_H

/*
 * analyze_compass_offsets
 *
 */

#include "analyzer.h"

class Analyzer_Compass_Offsets : public Analyzer {

public:
    Analyzer_Compass_Offsets(AnalyzerVehicle::Base *&vehicle) :
	Analyzer(vehicle)
    { }

    const char *name() const { return "Compass Offsets"; }
    const char *description() const {
        return "This test will FAIL if the compass offset parameters exceed thresholds";
    }

    bool configure(INIReader *config);
    // void handle_decoded_message(uint64_t T, mavlink_param_value_t &param) override;
    void evaluate() override;

private:
    // we rely on these changing:
    uint64_t modtime_compass_ofs[3] = {
        (uint64_t)-1,
        (uint64_t)-1,
        (uint64_t)-1
    };

    const uint16_t warn_offset = 100;
    const uint16_t fail_offset = 200;

    enum compass_offset_status {
        compass_offset_warn = 17,
        compass_offset_fail,
        compass_offset_zero,
        compass_offset_ok,
    };
    struct compass_offset_result {
        uint64_t timestamp;
        double len;
        double lens[3];
        compass_offset_status status;
    };
    bool new_compass_results();

    void do_add_severity_score(struct compass_offset_result result);

    #define MAX_COMPASS_OFFSET_RESULTS 100
    uint8_t compass_offset_results_offset = 0;
    compass_offset_result compass_offset_results[MAX_COMPASS_OFFSET_RESULTS];
    bool compass_offset_results_overrun = false;
    
    void add_evidence(Json::Value &root, compass_offset_result result);
    uint32_t results_json_compass_offsets_status_reason(char *buf, const uint32_t buflen, compass_offset_result result);
    const char * results_json_compass_offsets_status_string(compass_offset_result result);
    uint32_t results_json_compass_offsets(char *buf, uint32_t buflen);
    void results_json_results(Json::Value &root);

    void results_json_compass_offsets(Json::Value &root);
};

#endif
