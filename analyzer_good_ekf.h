#ifndef ANALYZER_GOOD_EKF_H
#define ANALYZER_GOOD_EKF_H

/*
 * analyzer_good_ekf
 *
 */

#include "analyzer.h"

class Analyzer_Good_EKF : public Analyzer {

public:
    Analyzer_Good_EKF(AnalyzerVehicle::Base *&vehicle) :
	Analyzer(vehicle),
        velocity_variance{
            name: "velocity_variance",
            threshold_warn: 0.5f,
            threshold_fail: 1.0f,
        },
        pos_horiz_variance{
            name: "pos_horiz_variance",
            threshold_warn: 0.5f,
            threshold_fail: 1.0f,
        },
        pos_vert_variance{
            name: "pos_vert_variance",
            threshold_warn: 0.5f,
            threshold_fail: 1.0f,
        },
        compass_variance{
            name: "compass_variance",
            threshold_warn: 0.5f,
            threshold_fail: 1.0f,
        },
        terrain_alt_variance{
            name: "terrain_alt_variance",
            threshold_warn: 0.5f,
            threshold_fail: 1.0f,
        },

        result_velocity_variance{
            variance: &velocity_variance,
            T_start: 0
        },
        result_pos_horiz_variance({
            variance: &pos_horiz_variance,
            T_start: 0
        }),
        result_pos_vert_variance({
            variance: &pos_vert_variance,
            T_start: 0
        }),
        result_compass_variance({
            variance: &compass_variance,
            T_start: 0
        }),
        result_terrain_alt_variance({
            variance: &terrain_alt_variance,
            T_start: 0
        })
    { }


    const char *name() const { return "Good EKF"; }
    const char *description() const {
        return "This test will FAIL if EKF variances exceed thresholds, or if the EKF status flags indicate errors";
    }

    bool configure(INIReader *config);

    void results_json_results(Json::Value &root);

    struct ekf_variance {
        const char *name;
        double threshold_warn;
        double threshold_fail;
    };
    struct ekf_variance_result {
        struct ekf_variance *variance;
        uint64_t T_start;
        uint64_t T_stop;
        double max;
    };

    struct ekf_variance_result *variance_result(std::string name) {
        return _results[name];
    }
private:
    bool seen_ekf_packets = false;

    

    struct ekf_variance velocity_variance;
    struct ekf_variance pos_horiz_variance;
    struct ekf_variance pos_vert_variance;
    struct ekf_variance compass_variance;
    struct ekf_variance terrain_alt_variance;

    struct ekf_variance_result result_velocity_variance;
    struct ekf_variance_result result_pos_horiz_variance;
    struct ekf_variance_result result_pos_vert_variance;
    struct ekf_variance_result result_compass_variance;
    struct ekf_variance_result result_terrain_alt_variance;

    std::map<const std::string, struct ekf_variance_result*> _results = {
        { "velocity", &result_velocity_variance },
        { "pos_horiz", &result_pos_horiz_variance },
        { "pos_vert", &result_pos_vert_variance },
        { "compass", &result_compass_variance },
        { "terrain_alt", &result_terrain_alt_variance }
    };

    #define MAX_VARIANCE_RESULTS 100
    uint8_t next_result_variance = 0;
    struct ekf_variance_result result_variance[MAX_VARIANCE_RESULTS] = { };

    struct ekf_flags_result {
        uint64_t T_start;
        uint64_t T_stop;
        uint16_t flags;
    };

    #define MAX_FLAGS_RESULTS 100
    uint8_t next_result_flags = 0;
    struct ekf_flags_result result_flags[MAX_FLAGS_RESULTS] = { };

    void evaluate_variance(std::string name, double value);
    void evaluate_variances();
    void evaluate_flags();
    void evaluate() override;

    bool ekf_flags_bad(uint16_t flags);

    void close_variance_result(struct ekf_variance_result *result);
    void end_of_log(uint32_t packet_count) override;

    void results_json_results_do_variance(Json::Value &root, const struct ekf_variance_result *variance_result);
    void results_json_results_do_flags(Json::Value &root, const struct ekf_flags_result flags_result);
};

#endif


