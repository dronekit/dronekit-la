#ifndef ANALYZER_SENSOR_HEALTH_H
#define ANALYZER_SENSOR_HEALTH_H

/*
 * analyzer_sensor_health
 *
 */

#include "analyzer.h"
#include "data_sources.h"


class Analyzer_Sensor_Health_Result : public Analyzer_Result_Period {
public:
    Analyzer_Sensor_Health_Result(std::string sensor_name) :
        _sensor_name(sensor_name)
        { }
    const std::string sensor_name() const { return _sensor_name; }
    void to_json(Json::Value &root) const;

private:
    const std::string _sensor_name;
};

class Analyzer_Sensor_Health : public Analyzer {

public:
    Analyzer_Sensor_Health(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
    Analyzer(vehicle, data_sources)
    { }


    const std::string name() const override { return "Sensor Health"; }
    const std::string description() const override {
        return "A UAV can self-assess its sensors' health.  This test will FAIL if any sensor is detected as failed.";
    }

    bool configure(INIReader *config) override;

private:

    std::map<const std::string, bool> _sensor_health = { };
    std::map<const std::string, Analyzer_Sensor_Health_Result*> _results = { };

    void evaluate() override;

    void close_result(std::string name);
    void open_result(std::string name);

    void end_of_log(uint32_t packet_count) override;
};

#endif


