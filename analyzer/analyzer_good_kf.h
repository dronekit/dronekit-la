#pragma once

/*
 * analyzer_good_kf
 *
 */

#include "analyzer.h"
#include "data_sources.h"


class kf_variance {
public:
    const char *name;
    double threshold_warn;
    double threshold_fail;
};

class Analyzer_Good_KF_Result_Variance : public Analyzer_Result_Period {
public:
    void set_max(double max) { _max = max; }
    double max() const { return _max; }
    void set_variance(kf_variance *variance) { _variance = variance; }
    kf_variance *variance() const { return _variance; }

private:
    kf_variance *_variance;
    double _max;
};


class Analyzer_Good_KF_Result_Flags : public Analyzer_Result_Period {
public:
    void set_flags(uint16_t flags) { _flags = flags; }
    uint16_t flags() { return _flags; }
private:
    uint16_t _flags;
};

class Analyzer_Good_KF : public Analyzer {

public:

    using Analyzer::Analyzer;

    virtual const std::string shortname_lc() const = 0;
    virtual const std::string shortname() const = 0;
    virtual uint64_t variance_T(std::string name) const = 0;
    virtual std::map<const std::string, double> variances() = 0;
    virtual uint64_t flags_T() const = 0;
    virtual uint16_t flags() const = 0;

    bool configure(INIReader *config) override;

private:

// allow designated initialisers:
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
    std::map<const std::string, kf_variance> _variances = {
        { "velocity", {
            .name = "velocity_variance",
            .threshold_warn = 0.5f,
            .threshold_fail = 1.0f,
            }
        },
        { "pos_horiz", {
            .name = "pos_horiz_variance",
            .threshold_warn = 0.5f,
            .threshold_fail = 1.0f,
            }
        },
        { "pos_vert", {
            .name = "pos_vert_variance",
            .threshold_warn = 0.5f,
            .threshold_fail = 1.0f,
            }
        },
        { "compass", {
            .name = "compass_variance",
            .threshold_warn = 0.5f,
            .threshold_fail = 1.0f,
            }
        },
        { "terrain_alt", {
            .name = "terrain_alt_variance",
            .threshold_warn = 0.5f,
            .threshold_fail = 1.0f,
            }
        }
    };
#pragma GCC diagnostic pop

    std::map<const std::string, Analyzer_Good_KF_Result_Variance*> _results = {
        { "velocity", NULL },
        { "pos_horiz", NULL },
        { "pos_vert", NULL },
        { "compass", NULL },
        { "terrain_alt", NULL }
    };

    void evaluate_variance(class kf_variance &variance, double value);
    void evaluate_variances();
    void evaluate_flags();
    void evaluate() override;

    void close_variance_result(const std::string name);

    Analyzer_Good_KF_Result_Flags *_result_flags = NULL;
    void close_result_flags();
    void open_result_flags(uint16_t flags);
    bool flags_bad(uint16_t flags);

    void end_of_log(uint32_t packet_count) override;

};
