#ifndef ANALYZER_PARAMETERS_H
#define ANALYZER_PARAMETERS_H

/*
 * analyzer_parameters
 *
 * Generally other analyzers should cover bad configuration via
 * parameters.  Until we have an analyzer which seems to cover a
 * parameter value check, it gets thrown in here
 */

#include "analyzer.h"

/// @brief Simple Limits that can be placed on a parameter
class Param_Constraint_Trivial {

public:


    Param_Constraint_Trivial(const std::string description,
                             const double min, const bool min_used,
                             const double max, const bool max_used,
                             const double bad_default,
                             const bool bad_default_used) :
        _description(description),
        _min(min),
        _min_used(min_used),
        _max(max),
        _max_used(max_used),
        _bad_default(bad_default),
        _bad_default_used(bad_default_used)
        { }

    const std::string description() const { return _description; }

    double min() const { return _min; }
    double max() const { return _max; }
    double bad_default() const { return _bad_default; }

    bool min_ok(const double value) const { return (!_min_used || value >= _min); }
    bool max_ok(const double value) const { return (!_max_used || value <= _max); }
    bool default_ok(const double value) const {
        return (!_bad_default_used || !is_equal(value,_bad_default));
    }

    bool value_ok(const double value) const {
        return ((!_min_used || min_ok(value)) &&
                (!_max_used || max_ok(value)) &&
                (!_bad_default_used || default_ok(value)));
    }

private:

    const std::string _description;

    double _min;
    bool _min_used;
    double _max;
    bool _max_used;
    double _bad_default;
    bool _bad_default_used;

};

const std::map<const std::string, Param_Constraint_Trivial> trivial_constraints = {
    { "ANGLE_MAX", { "Craft's maximum roll and pitch angle too small.  ANGLE_MAX is measured in centidegrees.  Craft have been lost when the operator attempted to configure this in degrees, meaning the autopilot could not change aircraft attitude.", 500, true, 0, false, 0, false } },
    { "BATT_AMP_PERVOLT", { "Calibration value for the current sensor is incorrect.  Power bricks vary in their measurements.  Unless calibrated, a true representation of the craft's remaining power will not be available.", 0, false, 0, false, 17, true } },
};


class Analyzer_Parameters_Result : public Analyzer_Result_Period{
public:
    Analyzer_Parameters_Result() :
        Analyzer_Result_Period()
        { }

    void set_value(const double value) { _value = value; }
    double value() { return _value; }

private:

    double _value;
};

class Analyzer_Parameters : public Analyzer {

public:

    Analyzer_Parameters(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
    Analyzer(vehicle,data_sources)
    { }

    const std::string name() const override { return "Parameters"; }
    const std::string description() const override {
        return "Autopilots store configuration settings known as 'parameters'. This test detects some common configuration issues present in these parameters.";
    }

    void evaluate_trivial();
    void evaluate() override;

    void end_of_log(const uint32_t packet_count) override;

    void open_result_trivial(const std::string name, const double value, const Param_Constraint_Trivial &constraints);
    void update_result_trivial(const std::string name, const double value, const Param_Constraint_Trivial &constraints);
    void close_result_trivial(const std::string name);

    void evaluate_log_bitmask();

protected:

    bool configure(INIReader *config) override;

private:

    std::map<const std::string, Analyzer_Parameters_Result*> _open_results;

    bool _enable_ANGLE_MAX_check;

    const float log_bitmask_apm_default = 830;
    Analyzer_Parameters_Result *_log_bitmask_bad_apm = nullptr;
};

#endif
