#ifndef ANALYZER_GOOD_XKF_H
#define ANALYZER_GOOD_XKF_H

/*
 * analyzer_good_xkf
 *
 */

#include "analyzer.h"
#include "data_sources.h"
#include "analyzer_good_kf.h"

class Analyzer_Good_XKF : public Analyzer_Good_KF {

public:

    // borrow the constructor:
    using Analyzer_Good_KF::Analyzer_Good_KF;

    const std::string name() const override { return "Good EKF3"; }
    const std::string description() const override {
        return "ArduPilot's third Extended Kalman Filter (EKF3) has many built-in checks to ensure that it is functioning correctly.  This test will FAIL or WARN if EKF variances exceed the respective thresholds, or FAIL if the EKF status flags indicate errors.";
    }

    const std::string shortname_lc() const override { return "xkf"; }
    const std::string shortname() const override { return "XKF"; }

    uint64_t variance_T(std::string name) const override {
        return _vehicle->xkf_variance_T(name);
    }
    uint64_t flags_T() const override {
        return _vehicle->xkf_flags_T();
    }
    uint16_t flags() const override {
        return _vehicle->xkf_flags();
    }

    std::map<const std::string, double> variances() {
        return _vehicle->xkf_variances();
    }

private:

};

#endif


