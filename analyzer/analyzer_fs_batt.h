#ifndef ANALYZER_FS_BATT_H
#define ANALYZER_FS_BATT_H

/*
 * analyzer_fs_batt
 *
 */

#include "analyzer.h"

class Analyzer_FS_Batt_Result : public Analyzer_Result_Period {

public:

    Analyzer_FS_Batt_Result() :
        Analyzer_Result_Period()
        { }

    void set_fs_batt_voltage(const double value) { _fs_batt_voltage = value; }
    double fs_batt_voltage() const { return _fs_batt_voltage; }

    void set_delta_max(const double value) { _delta_max = value; }
    double delta_max() const { return _delta_max; }

private:

    double _delta_max;
    double _fs_batt_voltage;

};


class Analyzer_FS_Batt : public Analyzer {

public:

    Analyzer_FS_Batt(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
    Analyzer(vehicle,data_sources)
        {}

    void evaluate() override;

    const std::string name() const override { return "FS Batt"; }
    const std::string description() const override {
        return "A lower-limit may be specified for the the UAV battery voltage.  This test will FAIL if this value is not within a certain delta of the actual battery voltage.  This can happen e.g. if a craft is switched from running on 3S batteries to 4S batteries.";
    }
    bool configure(INIReader *config) override;

    void end_of_log(const uint32_t packet_count) override;

private:

    Analyzer_FS_Batt_Result *_result;

    void open_result();
    void close_result();
    void update_result();

    bool fs_batt_bad();
};

#endif


