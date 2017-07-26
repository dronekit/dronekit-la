#ifndef ANALYZE_COMPASS_VECTOR_LENGTH_H
#define ANALYZE_COMPASS_VECTOR_LENGTH_H

/*
 * analyze_attitudecontrol
 *
 */

#include "analyzer.h"

#include <functional>

class Analyzer_Compass_Vector_Length_Result_Short : public Analyzer_Result_Period {
    friend class Analyzer_Compass_Vector_Length;
public:
    Analyzer_Compass_Vector_Length_Result_Short(std::string name) :
        _name(name) { }
    const std::string name() { return _name; };
    double length_min() { return _length_min; }

private:
    double _length_min = -1;
    const std::string _name;
};


class Analyzer_Compass_Vector_Length_Result_Long : public Analyzer_Result_Period {
    friend class Analyzer_Compass_Vector_Length;

public:

    Analyzer_Compass_Vector_Length_Result_Long(std::string name) :
        _name(name),
        _length_max(0)
        { }

    const std::string name() { return _name; };
    double length_max() { return _length_max; }

private:
    const std::string _name;
    double _length_max;
};


class Analyzer_Compass_Vector_Length : public Analyzer {

public:

    Analyzer_Compass_Vector_Length(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
    Analyzer(vehicle,data_sources)
    { }

    const std::string name() const override { return "Compass Vector Length"; }
    const std::string description() const override {
        return "The strength and direction of the Earth's magnetic field should be relatively constant and lie within certain thresholds.  This test will FAIL or WARN if the compass vector length exceeds the respective threshold.  Possible causes include flying near large metal objects.";
    }

    bool configure(INIReader *config) override;

    void evaluate_compass_field_length(AnalyzerVehicle::Compass *compass);
    void evaluate_compass(AnalyzerVehicle::Compass *compass);
    void evaluate() override;

    void for_each_compass(const std::function <void (AnalyzerVehicle::Compass*)> f);

private:
    float length_short_warn = 120.0f;
    float length_short_fail = 100.0f;
    float length_long_warn = 550.0f;
    float length_long_fail = 600.0f;

    float delta_warn_percent = 25.0f;
    float delta_fail_percent = 35.0f;
    
    uint32_t duration_min = 250000; // microseconds

    void end_of_log(uint32_t packet_count) override;

    std::map<const std::string, double> _longest_vector_length_seen = { };
    std::map<const std::string, double> _shortest_vector_length_seen = { };
    void check_vector_delta(const std::string name,
                            double shortest,
                            double longest);
    std::map<const std::string, Analyzer_Compass_Vector_Length_Result_Short *>
    _result_short = { };
    void open_result_short(std::string name, double length);
    void update_result_short(Analyzer_Compass_Vector_Length_Result_Short *, double length);
    void close_result_short(Analyzer_Compass_Vector_Length_Result_Short *);

    void close_results_short();
    void close_results_long();

    std::map<const std::string, Analyzer_Compass_Vector_Length_Result_Long *>
       _result_long;
    void open_result_long(std::string name, double length);
    void update_result_long(Analyzer_Compass_Vector_Length_Result_Long *, double length);
    void close_result_long(Analyzer_Compass_Vector_Length_Result_Long *);

};

#endif
