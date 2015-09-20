#ifndef ANALYZER_ESTIMATE_DIVERGENCE_H
#define ANALYZER_ESTIMATE_DIVERGENCE_H

/*
 * analyzer_estimate_divergence
 *
 */

#include "analyzer.h"

#include "analyzer_util.h"

class Analyzer_Estimate_Divergence_Result : public Analyzer_Result_Period {
public:
    Analyzer_Estimate_Divergence_Result(std::string name) :
        Analyzer_Result_Period()
        {
            set_name(name);
        }

    void set_name(const std::string name) { _name = name; }
    std::string name() { return _name; }

    void set_max_delta(const double delta) { _max_delta = delta; }
    const double max_delta() { return _max_delta; }

    void set_delta_threshold(const double delta) { _delta_threshold = delta; }
    const double delta_threshold() { return _delta_threshold; }

    virtual void to_json(Json::Value &root) const override;
private:
    std::string _name = "";
    double _max_delta;
    double _delta_threshold;
};

class Analyzer_Estimate_Divergence : public Analyzer {

public:

    Analyzer_Estimate_Divergence(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
	Analyzer(vehicle,data_sources)
    { }

    virtual const std::string estimate_name() const = 0;

    virtual const std::string name() const override {
        return string_format("%s Estimate Divergence", estimate_name().c_str());
    }
    virtual const std::string description() const override {
        return string_format("This test will FAIL if various craft's %s estimates diverge", estimate_name().c_str());
    }

    // void evaluate() override;

    // void open_result(std::string name, double delta);
    virtual void update_result_set_status(Analyzer_Estimate_Divergence_Result *result);
    void update_result(std::string name, double delta);
    void close_result(std::string name);
    virtual void close_result_add_evidence(Analyzer_Estimate_Divergence_Result *result);
    void end_of_log(const uint32_t packet_count) override;

protected:

    double delta_fail() {
        return _delta_fail;
    }
    double delta_warn() {
        return _delta_warn;
    }
    double delta_time_threshold() {
        return _delta_time_threshold;
    }

    const float _delta_warn = 4.0f;
    const float _delta_fail = 5.0f;

    const uint64_t _delta_time_threshold = 500000;

    Analyzer_Estimate_Divergence_Result *result_for_name(std::string name) {
        return _result[name];
    }
    void set_result_for_name(std::string name,
                             Analyzer_Estimate_Divergence_Result *result) {
        _result[name] = result;
    }
private:
    std::map<const std::string, Analyzer_Estimate_Divergence_Result*> _result;
};

#endif
