#include "analyzer_gps_fix.h"

#include "util.h"
#include "analyzer_util.h"

bool Analyzer_GPS_Fix::configure(INIReader *config) {
    if (!Analyzer::configure(config)) {
	return false;
    }

    _satellites_min = config->GetInteger("loganalyzer", "gpsfix::satellites_min", 5);
    _hdop_min = config->GetReal("loganalyzer", "gpsfix::hdop_min", 5.0f);
    

    return true;
}

void Analyzer_GPS_Fix::end_of_log(uint32_t packet_count UNUSED)
{
    if (_result != NULL) {
        close_result();
    }
}

bool Analyzer_GPS_Fix::gpsinfo_bad(AnalyzerVehicle::GPSInfo *gpsinfo) const
{
   if (gpsinfo->satellites() <= satellites_visible_threshold()) {
        return true;
    }
    if (gpsinfo->hdop() >= hdop_threshold()) {
        return true;
    }
    return false;
}

void Analyzer_GPS_Fix::add_firstfixtime_result(AnalyzerVehicle::GPSInfo *gpsinfo,
                                               uint64_t time_taken)
{
    Analyzer_GPS_FirstFixTime_Result *result =
        new Analyzer_GPS_FirstFixTime_Result(gpsinfo->name());
    result->set_reason("First 3D GPS Fix Acquired");
    result->add_evidence(string_format("first-fix-time=%f", (float)time_taken/1000000.0f));
    result->add_evidence(string_format("first-fix-time-units=seconds", time_taken));
    result->set_time_taken(time_taken);
    result->set_status(analyzer_status_ok);
    result->add_source(_data_sources.get(std::string("GPSINFO_FIXTYPE_") + result->name()));
    result->add_source(_data_sources.get(std::string("SYSTEM_TIME") + result->name()));
    result->set_T(_vehicle->T());
    add_result(result);
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

    if (!_first_3D_fix_found) {
        if (gpsinfo->fix_type() >= 3) {
            // new fix!
            if (_vehicle->time_since_boot_T() == 0) {
                // we have a fix, but no system boot time.  This can
                // happen on a tlog when you connect to a system which
                // has been up for some time; you can receive your GPS
                // messages before your system_time messages.
            } else {
                if (!_non_3dfix_seen) { // 
                    // we probably came in when the UAV had already
                    // been running for some time.  Ignore this
                    // result.
                    _first_3D_fix_found = true;
                } else {
                    _first_3D_fix_found = true;
                    _first_3D_fix_T = _vehicle->time_since_boot();
                    add_firstfixtime_result(gpsinfo, _vehicle->time_since_boot());
                }
            }
        } else {
            _non_3dfix_seen = true;
        }
    } else {
        if (_vehicle->time_since_boot() < _first_3D_fix_T) {
            // reboot; chance of getting another result:
            _first_3D_fix_found = false;
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
    _result->add_evidence(string_format("HDop-threshold: %f", hdop_threshold()));

    _result->add_source(_data_sources.get(std::string("GPSINFO_") + _result->name()));

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
