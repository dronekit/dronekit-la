#ifndef ANALYZE_COMPASS_OFFSETS_H
#define ANALYZE_COMPASS_OFFSETS_H

/*
 * analyze_compass_offsets
 *
 */

#include "analyzer.h"

class Analyzer_Compass_Offsets : public Analyzer {

public:
    Analyzer_Compass_Offsets(int fd, struct sockaddr_in *sa, AnalyzerVehicle::Base *&vehicle) :
	Analyzer(fd, sa, vehicle),
        modtime_compass_ofs{ },
        warn_offset(100),
        fail_offset(200),
        compass_offset_results_offset(0)
    { }

    const char *name() const { return "Compass Offsets"; }
    const char *description() const {
        return "This test will FAIL if the compass offset parameters exceed thresholds";
    }

    bool configure(INIReader *config);
    void handle_decoded_message(uint64_t T, mavlink_param_value_t &param) override;
    void evaluate(uint64_t T);

private:
    uint64_t modtime_compass_ofs[3];

    const uint16_t warn_offset;
    const uint16_t fail_offset;

    enum compass_offset_status {
        compass_offset_warn = 17,
        compass_offset_fail,
        compass_offset_zero,
        compass_offset_ok,
    };
    struct compass_offset_result {
        uint64_t timestamp;
        double len;
        compass_offset_status status;
    };
    bool new_compass_results();

    void do_add_evilness(struct compass_offset_result result);

    #define MAX_COMPASS_OFFSET_RESULTS 100
    uint8_t compass_offset_results_offset;
    compass_offset_result compass_offset_results[MAX_COMPASS_OFFSET_RESULTS];
    bool compass_offset_results_overrun;
    
    void add_evidence(Json::Value &root, compass_offset_result result);
    uint32_t results_json_compass_offsets_status_reason(char *buf, const uint32_t buflen, compass_offset_result result);
    const char * results_json_compass_offsets_status_string(compass_offset_result result);
    uint32_t results_json_compass_offsets(char *buf, uint32_t buflen);
    void results_json_results(Json::Value &root);

    void results_json_compass_offsets(Json::Value &root);
};

#endif
