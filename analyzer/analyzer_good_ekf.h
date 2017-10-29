#ifndef ANALYZER_GOOD_EKF_H
#define ANALYZER_GOOD_EKF_H

/*
 * analyzer_good_ekf
 *
 */

#include "analyzer.h"
#include "data_sources.h"
#include "analyzer_good_kf.h"

class Analyzer_Good_EKF : public Analyzer_Good_KF {

public:

    // borrow the constructor:
    using Analyzer_Good_KF::Analyzer_Good_KF;

    const std::string name() const override { return "Good EKF"; }
    const std::string description() const override {
        return "The Extended Kalman Filter (EKF) has many built-in checks to ensure that it is functioning correctly.  This test will FAIL or WARN if EKF variances exceed the respective thresholds, or FAIL if the EKF status flags indicate errors.";
    }

    const std::string shortname_lc() const override { return "ekf"; }
    const std::string shortname() const override { return "EKF"; }

    uint64_t variance_T(std::string name) const override {
        return _vehicle->ekf_variance_T(name);
    }
    uint64_t flags_T() const override {
        return _vehicle->ekf_flags_T();
    }
    uint16_t flags() const override {
        return _vehicle->ekf_flags();
    }

    std::map<const std::string, double> variances() {
        return _vehicle->ekf_variances();
    }

private:

};

#endif


