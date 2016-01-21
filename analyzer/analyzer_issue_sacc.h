#ifndef ANALYZER_ISSUE_SACC_H
#define ANALYZER_ISSUE_SACC_H

/*
 * analyzer_issue_sacc
 *
 */

#include "analyzer.h"
#include "data_sources.h"


class Analyzer_Issue_Sacc_Result : public Analyzer_Result_Period {
public:
    Analyzer_Issue_Sacc_Result(std::string name) :
        _name(name)
              { };
    void set_sacc(double sacc) { _sacc = sacc; }
    double sacc() const { return _sacc; }
    const std::string name() { return _name; }
private:
    const std::string _name;
    double _sacc;
};

class Analyzer_Issue_Sacc : public Analyzer {

public:
    Analyzer_Issue_Sacc(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
    Analyzer(vehicle, data_sources)
    { }


    const std::string name() const override { return "sAcc Issue"; }
    const std::string description() const override {
        return "This is an example of a cut-down analyzer which looks for a specific issue.";
    }

    bool configure(INIReader *config) override;

private:

    void evaluate() override;
    void evaluate_gps(AnalyzerVehicle::GPSInfo *gpsinfo);

    Analyzer_Issue_Sacc_Result *_result = NULL;
    void close_result();
    void open_result(AnalyzerVehicle::GPSInfo *gpsinfo);
    void update_result(AnalyzerVehicle::GPSInfo *gpsinfo);

    void end_of_log(uint32_t packet_count) override;

    double sacc_threshold_fail() const { return _sacc_threshold_fail; }

    bool gpsinfo_bad(AnalyzerVehicle::GPSInfo *gpsinfo) const;

    float _sacc_threshold_fail = 1.5f;
    bool _seen_fix = false;
};

#endif


