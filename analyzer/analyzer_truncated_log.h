#ifndef ANALYZER_TRUNCATED_LOG_H
#define ANALYZER_TRUNCATED_LOG_H

/*
 * analyzer_truncated_log
 *
 */

#include "analyzer.h"

class Analyzer_Truncated_Log_Result : public Analyzer_Result_Summary {
public:
    Analyzer_Truncated_Log_Result() :
        Analyzer_Result_Summary()
        { }

    void set_last_altitude(double altitude) { _alt_last = altitude; }
    double last_altitude() { return _alt_last; }

    void set_takeoff_altitude(double altitude) {
        _alt_takeoff = altitude;
    }
    double takeoff_altitude() { return _alt_takeoff; }

private:
    double _alt_last;
    double _alt_takeoff;
};


class Analyzer_Truncated_Log : public Analyzer {

public:

    Analyzer_Truncated_Log(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
    Analyzer(vehicle,data_sources)
    { }

    const std::string name() const override { return "Truncated Log"; }
    const std::string description() const override {
        return "A log should not end while the vehicle appears to be moving under its own power.  This test will FAIL if the vehicle still appears to be moving when the log ends.";
    }
    bool configure(INIReader *config) override;

    void evaluate() override;
    void end_of_log(const uint32_t packet_count) override;

private:

    bool _old_is_flying = false;
    double _lowest_voltage = 100000000.0f; // volts
    double _highest_voltage = 0; // volts

    double max_last_relative_altitude = 5.0f; // metres
    double _min_low_voltage; // volts
    double _max_voltage_delta; // volts

    Analyzer_Truncated_Log_Result _result;
};

#endif


