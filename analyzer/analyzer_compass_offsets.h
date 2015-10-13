#ifndef ANALYZE_COMPASS_OFFSETS_H
#define ANALYZE_COMPASS_OFFSETS_H

/*
 * analyze_compass_offsets
 *
 */

#include "analyzer.h"

#include "Vector3f.h"

class Analyzer_Compass_Offsets_Result : public Analyzer_Result_Event {
public:
    void set_lens(double x, double y, double z) {
        _lens[0] = x;
        _lens[1] = y;
        _lens[2] = z;
    }
    Vector3f lens()& { return _lens; }

private:
    // FIXME: scope
    Vector3f _lens;
};

class Analyzer_Compass_Offsets : public Analyzer {

public:

    Analyzer_Compass_Offsets(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources, const std::string param_extra_string) :
	Analyzer(vehicle,data_sources),
        _param_extra_string(param_extra_string)
    { }

    const std::string name() const override { return "Compass Offsets"; }
    const std::string description() const override {
        return "This test will FAIL if the compass offset parameters exceed thresholds";
    }

    bool configure(INIReader *config) override;

    void evaluate() override;

    void end_of_log(uint32_t packet_count) override;

private:
    // we rely on these changing:
    uint64_t modtime_compass_ofs[3] = {
        (uint64_t)-1,
        (uint64_t)-1,
        (uint64_t)-1
    };
    
    bool compass_use();
    bool new_compass_results();

#define MAX_COMPASS_OFFSET_RESULTS 100
    bool compass_offset_results_overrun = false;

    bool _old_compass_use = false;

    const std::string _param_extra_string;

    double warn_offset = 100.0f;
    double fail_offset = 200.0f;
};

#endif
