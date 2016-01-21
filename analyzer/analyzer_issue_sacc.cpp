#include "analyzer_issue_sacc.h"

#include "util.h"
#include "analyzer_util.h"

bool Analyzer_Issue_Sacc::configure(INIReader *config) {
    if (!Analyzer::configure(config)) {
	return false;
    }

    return true;
}

void Analyzer_Issue_Sacc::end_of_log(uint32_t packet_count UNUSED)
{
    if (_result != NULL) {
        close_result();
    }
}

bool Analyzer_Issue_Sacc::gpsinfo_bad(AnalyzerVehicle::GPSInfo *gpsinfo) const
{
    if (gpsinfo->sacc() >= sacc_threshold_fail()) {
        return true;
    }
    return false;
}

void Analyzer_Issue_Sacc::evaluate_gps(AnalyzerVehicle::GPSInfo *gpsinfo)
{
    if (!_seen_fix) {
        if (gpsinfo->fix_type() < 3) {
            return;
        }
        _seen_fix = true;
    }

    bool results_bad = gpsinfo_bad(gpsinfo);
    if (_result == NULL) {
        if (results_bad) {
            open_result(gpsinfo);
        }
    } else {
        if (results_bad) {
            update_result(gpsinfo);
        } else {
            close_result();
        }
    }
}


void Analyzer_Issue_Sacc::evaluate()
{
    std::map<const std::string, AnalyzerVehicle::GPSInfo*> gpss = _vehicle->gpsinfos();

    for (std::map<const std::string, AnalyzerVehicle::GPSInfo*>::const_iterator it = gpss.begin();
         it != gpss.end();
         ++it) {
        std::string name = (*it).first;
        if (gpss.count(name) == 0) {
            // set a flag and warn?
            fprintf(stderr, "Unknown gps %s\n", name.c_str());
            continue;
        }
        AnalyzerVehicle::GPSInfo *gps = gpss[name];
        evaluate_gps(gps);
    }
}

void Analyzer_Issue_Sacc::close_result()
{
    _result->set_T_stop(_vehicle->T());

    _result->add_evidence(string_format("sAcc: %.2f", _result->sacc()));
    _result->add_evidence(string_format("sAcc-fail-threshold: %f", sacc_threshold_fail()));

    if (_result->sacc() > sacc_threshold_fail()) {
        _result->increase_severity_score(20);
    }

    _result->add_source(_data_sources.get(std::string("GPSINFO_") + _result->name()));
    add_result(_result);
    _result = NULL;
}

void Analyzer_Issue_Sacc::update_result(AnalyzerVehicle::GPSInfo *gpsinfo)
{
    if (_result->sacc() < gpsinfo->sacc()) {
        _result->set_sacc(gpsinfo->sacc());
    }
}

void Analyzer_Issue_Sacc::open_result(AnalyzerVehicle::GPSInfo *gpsinfo)
{
    _result = new Analyzer_Issue_Sacc_Result(gpsinfo->name());
    _result->set_T_start(_vehicle->T());

    _result->set_status(analyzer_status_fail);
    _result->set_reason("sAcc is Bad");

    _result->set_sacc(gpsinfo->sacc());
}
