#ifndef ANALYZE_EVER_FLEW_H
#define ANALYZE_EVER_FLEW_H

/*
 * analyze_ever_flew
 *
 */

#include "analyzer.h"
#include "analyzervehicle_copter.h"

class Analyzer_Ever_Flew_Result : public Analyzer_Result_Summary {
public:
    Analyzer_Ever_Flew_Result() :
        Analyzer_Result_Summary()
        { }

    uint64_t pass_timestamp() const { return _pass_timestamp; }
    void set_pass_timestamp(uint64_t timestamp) { _pass_timestamp = timestamp; }

    bool ever_armed() const { return _ever_armed; }
    void set_ever_armed(bool value) { _ever_armed = value; }

    bool servos_past_threshold() const { return _servos_past_threshold; }
    void set_servos_past_threshold(bool value) { _servos_past_threshold = value; }

    void to_json(Json::Value &root) const override;

private:
    uint64_t _pass_timestamp = 0;
    bool _ever_armed = false;
    bool _servos_past_threshold = false;
};

class Analyzer_Ever_Flew : public Analyzer {
public:

    Analyzer_Ever_Flew(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
    Analyzer(vehicle,data_sources)
    {
        _result.set_status(analyzer_status_fail);
        _result.set_reason("The vehicle never seemed to take off");
        _result.add_source(_data_sources.get("ARMING"));
    }

    const std::string name() const { return "Ever Flew"; }
    const std::string description() const {
        return "Determining whether a vehicle has ever flown in a log is done heuristically based on things like motor speeds.  This test will FAIL if the craft did not ever seem to fly.";
    }

    bool configure(INIReader *config) override;

    void end_of_log(const uint32_t packet_count) override;

    uint64_t total_flight_time() const { return _total_flight_time; };

private:

    Analyzer_Ever_Flew_Result _result;

    void evaluate() override;

    uint64_t _total_flight_time = 0;
    bool _was_flying = false;
    uint64_t _fly_start_time = 0;

    bool _added_servo_output = false;
};

#endif
