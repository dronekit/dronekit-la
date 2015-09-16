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

    void set_takeoff_altitude(double altitude) { _alt_takeoff = altitude; }
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

    const char *name() const override { return "Brownout"; }
    const char *description() const override {
        return "This test will FAIL if the craft appears to lose onboard power during a flight";
    }
    bool configure(INIReader *config) override;

    void evaluate() override;
    void end_of_log(const uint32_t packet_count) override;

private:
    bool seen_packets = false;

    bool _old_is_flying = false;

    const double max_last_relative_altitude = 5.0f; // metres

    Analyzer_Brownout_Result _result;
};

#endif


