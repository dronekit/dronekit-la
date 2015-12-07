#ifndef ANALYZER_GOOD_EKF_H
#define ANALYZER_GOOD_EKF_H

/*
 * analyzer_good_ekf
 *
 */

#include "analyzer.h"
#include "data_sources.h"


class ekf_variance {
public:
    const char *name;
    double threshold_warn;
    double threshold_fail;
};

class Analyzer_Good_EKF_Result_Variance : public Analyzer_Result_Period {
public:
    void set_max(double max) { _max = max; }
    double max() const { return _max; }
    void set_variance(ekf_variance *variance) { _variance = variance; }
    ekf_variance *variance() const { return _variance; }

private:
    ekf_variance *_variance;
    double _max;
};


class Analyzer_Good_EKF_Result_Flags : public Analyzer_Result_Period {
public:
    void set_flags(uint16_t flags) { _flags = flags; }
    uint16_t flags() { return _flags; }
private:
    uint16_t _flags;
};

class Analyzer_Good_EKF : public Analyzer {

public:
    Analyzer_Good_EKF(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
	Analyzer(vehicle, data_sources)
    { }


    const std::string name() const override { return "Good EKF"; }
    const std::string description() const override {
        return "The Extended Kalman Filter (EKF) has many built-in checks to ensure that it is functioning correctly.  This test will FAIL or WARN if EKF variances exceed the respective thresholds, or FAIL if the EKF status flags indicate errors.";
    }

    bool configure(INIReader *config) override;

private:

// allow designated initialisers:
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
    std::map<const std::string, ekf_variance> _variances = {
        { "velocity", {
            name: "velocity_variance",
            threshold_warn: 0.5f,
            threshold_fail: 1.0f,
            }
        },
        { "pos_horiz", {
            name: "pos_horiz_variance",
            threshold_warn: 0.5f,
            threshold_fail: 1.0f,
            }
        },
        { "pos_vert", {
            name: "pos_vert_variance",
            threshold_warn: 0.5f,
            threshold_fail: 1.0f,
            }
        },
        { "compass", {
            name: "compass_variance",
            threshold_warn: 0.5f,
            threshold_fail: 1.0f,
            }
        },
        { "terrain_alt", {
            name: "terrain_alt_variance",
            threshold_warn: 0.5f,
            threshold_fail: 1.0f,
            }
        }
    };
#pragma GCC diagnostic pop

    std::map<const std::string, Analyzer_Good_EKF_Result_Variance*> _results = {
        { "velocity", NULL },
        { "pos_horiz", NULL },
        { "pos_vert", NULL },
        { "compass", NULL },
        { "terrain_alt", NULL }
    };

    void evaluate_variance(struct ekf_variance &variance, double value);
    void evaluate_variances();
    void evaluate_flags();
    void evaluate() override;

    void close_variance_result(const std::string name);

    Analyzer_Good_EKF_Result_Flags *_result_flags = NULL;
    void close_result_flags();
    void open_result_flags(uint16_t flags);
    bool ekf_flags_bad(uint16_t flags);

    void end_of_log(uint32_t packet_count) override;

};

#endif


