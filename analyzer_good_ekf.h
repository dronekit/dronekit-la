#ifndef ANALYZER_GOOD_EKF_H
#define ANALYZER_GOOD_EKF_H

/*
 * analyzer_good_ekf
 *
 */

#include "analyzer.h"

class Analyzer_Good_EKF : public Analyzer {

public:
    Analyzer_Good_EKF(int fd, struct sockaddr_in *sa, AnalyzerVehicle::Base *&vehicle) :
	Analyzer(fd, sa, vehicle),
        seen_ekf_packets(false),
        
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

        // just one result ATM:
        result_velocity_variance({ }),
        result_pos_horiz_variance({ }),
        result_pos_vert_variance({ }),
        result_compass_variance({ }),
        result_terrain_alt_variance({ })
    {
    }

    const char *name() { return "Good EKF"; }
    const char *description() {
        return "EKF variances remain within bounds";
    }

    bool configure(INIReader *config);
    void handle_decoded_message(uint64_t T, mavlink_ekf_status_report_t &ekf_status_report) override;
    bool has_failed();


    void results_json_results(Json::Value &root);

private:
    bool seen_ekf_packets;
    
    struct ekf_variance {
        const char *name;
        double threshold_warn;
        double threshold_fail;
    };

    struct ekf_variance velocity_variance;
    struct ekf_variance pos_horiz_variance;
    struct ekf_variance pos_vert_variance;
    struct ekf_variance compass_variance;
    struct ekf_variance terrain_alt_variance;

    struct ekf_variance_result {
        uint64_t T;
        double max;
    };

    struct ekf_variance_result result_velocity_variance;
    struct ekf_variance_result result_pos_horiz_variance;
    struct ekf_variance_result result_pos_vert_variance;
    struct ekf_variance_result result_compass_variance;
    struct ekf_variance_result result_terrain_alt_variance;

    void results_json_results_do_variance(Json::Value &root, const struct ekf_variance variance, const struct ekf_variance_result variance_result);
};

#endif


