#ifndef ANALYZER_POSITION_ESTIMATE_DIVERGENCE_H
#define ANALYZER_POSITION_ESTIMATE_DIVERGENCE_H

/*
 * analyzer_position_estimate_divergence
 *
 */

#include "analyzer.h"

class Analyzer_Position_Estimate_Divergence_Result : public Analyzer_Result_Period {
public:
    Analyzer_Position_Estimate_Divergence_Result(std::string name) :
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

    void to_json(Json::Value &root) const override;
private:
    std::string _name = "";
    double _max_delta;
    double _delta_threshold;
};

class Analyzer_Position_Estimate_Divergence : public Analyzer {

public:

    Analyzer_Position_Estimate_Divergence(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
	Analyzer(vehicle,data_sources)
    { }

    const std::string name() const override { return "Position Estimate Divergence"; }
    const std::string description() const override {
        return "This test will FAIL if various craft's position estimates diverge";
    }

    void evaluate_estimate(
        std::string name,
        AnalyzerVehicle::Position position,
        AnalyzerVehicle::Position oestimate);
    void evaluate() override;

    void open_result(std::string name, double delta);
    void update_result(std::string name, double delta);
    void close_result(std::string name);

    void end_of_log(const uint32_t packet_count) override;

    double total_distance_travelled() { return _total_distance_travelled; }
private:

    const float position_delta_warn = 4.0f;
    const float position_delta_fail = 5.0f;

    const uint64_t delta_time_threshold = 500000;

    std::map<const std::string, Analyzer_Position_Estimate_Divergence_Result*> _result;

    double _total_distance_travelled = 0;
    AnalyzerVehicle::Position prevpos = { };
};

#endif
