#include "analyzer_gps_fix.h"

#include "util.h"
#include "analyzer_util.h"

bool Analyzer_GPS_Fix::configure(INIReader *config) {
    if (!Analyzer::configure(config)) {
	return false;
    }
    return true;
}

void Analyzer_GPS_Fix::end_of_log(uint32_t packet_count)
{
    if (_result != NULL) {
        close_result();
    }
}

bool Analyzer_GPS_Fix::gpsinfo_bad(AnalyzerVehicle::GPSInfo *gpsinfo)
{
   if (gpsinfo->satellites() <= satellites_visible_threshold()) {
        return true;
    }
    if (gpsinfo->hdop() >= hdop_threshold()) {
        return true;
    }
    return false;
}

void Analyzer_GPS_Fix::evaluate_gps(AnalyzerVehicle::GPSInfo *gpsinfo)
{
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


void Analyzer_GPS_Fix::evaluate()
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

void Analyzer_GPS_Fix::close_result()
{
    _result->set_T_stop(_vehicle->T());
    _result->add_evidence(string_format("satellites-visible: %d", _result->satellites()));
    _result->add_evidence(string_format("HDop: %.2f", _result->hdop()));
    _result->add_evidence(string_format("satellites-visible-threshold: %d", satellites_visible_threshold()));
    _result->add_evidence(string_format("HDop: %f", hdop_threshold()));

    // _result->add_series(_data_sources.get(std::string("GPSINFO_") + _result->name()));

    _result->add_evilness(20);
    add_result(_result);
    _result = NULL;
}

void Analyzer_GPS_Fix::update_result(AnalyzerVehicle::GPSInfo *gpsinfo)
{
    if (_result->satellites() > gpsinfo->satellites()) {
        _result->set_satellites(gpsinfo->satellites());
    }
    if (_result->hdop() < gpsinfo->hdop()) {
        _result->set_hdop(gpsinfo->hdop());
    }
}

void Analyzer_GPS_Fix::open_result(AnalyzerVehicle::GPSInfo *gpsinfo)
{
    _result = new Analyzer_GPS_Fix_Result(gpsinfo->name());
    _result->set_T_start(_vehicle->T());
    _result->set_status(analyzer_status_fail);
    _result->set_satellites(gpsinfo->satellites());
    _result->set_hdop(gpsinfo->hdop());
}
