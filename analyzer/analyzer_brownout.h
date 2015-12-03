#ifndef ANALYZER_BROWNOUT_H
#define ANALYZER_BROWNOUT_H

/*
 * analyzer_brownout
 *
 */

#include "analyzer.h"

class Analyzer_Brownout_Result : public Analyzer_Result_Summary {
public:
    Analyzer_Brownout_Result() :
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


class Analyzer_Brownout : public Analyzer {

public:

    Analyzer_Brownout(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
	Analyzer(vehicle,data_sources)
    { }

    const std::string name() const override { return "Brownout"; }
    const std::string description() const override {
        return "A log should not end while the vehicle appears to be moving under its own power.  This test will FAIL if the vehicle still appears to be moving when the log ends.";
    }
    bool configure(INIReader *config) override;

    void evaluate() override;
    void end_of_log(const uint32_t packet_count) override;

private:
    bool seen_packets = false;

    bool _old_is_flying = false;

    double max_last_relative_altitude = 5.0f; // metres

    Analyzer_Brownout_Result _result;
};

#endif


