#include "analyzer_gps_fix.h"

#include "util.h"
#include "analyzer_util.h"

bool Analyzer_GPS_Fix::configure(INIReader *config) {
    if (!Analyzer::configure(config)) {
	return false;
    }

    _satellites_min = config->GetInteger("loganalyzer", "gpsfix::satellites_min", 5);
    _hdop_min = config->GetReal("loganalyzer", "gpsfix::hdop_min", 5.0f);
    _sacc_threshold_warn = config->GetReal("loganalyzer", "gpsfix::sacc::threshold_warn", 1.0f);
    _sacc_threshold_fail = config->GetReal("loganalyzer", "gpsfix::sacc::threshold_fail", 1.5f);

    return true;
}

void Analyzer_GPS_Fix::end_of_log(uint32_t packet_count UNUSED)
{
    close_results_ff();
    close_results();
}

bool Analyzer_GPS_Fix::gpsinfo_bad(AnalyzerVehicle::GPSInfo *gpsinfo) const
{
   if (gpsinfo->satellites() <= satellites_visible_threshold()) {
        return true;
    }
    if (gpsinfo->hdop() >= hdop_threshold()) {
        return true;
    }
    if (gpsinfo->sacc() >= sacc_threshold_warn()) {
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
    Analyzer_GPS_Fix_Result *result = _result[gpsinfo->name()];
    if (result == NULL) {
        if (results_bad) {
            open_result(gpsinfo);
        }
    } else {
        if (results_bad) {
            update_result(gpsinfo);
        } else {
            close_result(gpsinfo->name(), result);
        }
    }

    if (_result_ff[gpsinfo->name()] == nullptr) {
        _result_ff[gpsinfo->name()] = new FirstFixInfo();
    }
    FirstFixInfo *info = _result_ff[gpsinfo->name()];
    if (!info->first_3D_fix_found) {
        if (gpsinfo->fix_type() >= 3) {
            // new fix!
            if (_vehicle->time_since_boot_T() == 0) {
                // we have a fix, but no system boot time.  This can
                // happen on a tlog when you connect to a system which
                // has been up for some time; you can receive your GPS
                // messages before your system_time messages.
            } else {
                if (!info->non_3dfix_seen) { // 
                    // we probably came in when the UAV had already
                    // been running for some time.  Ignore this
                    // result.
                    info->first_3D_fix_found = true;
                    info->first_3D_fix_T = 0; // avoids warning in comparison, below
                } else {
                    info->first_3D_fix_found = true;
                    info->first_3D_fix_T = _vehicle->time_since_boot();
                    add_firstfixtime_result(gpsinfo, _vehicle->time_since_boot());
                }
            }
        } else {
            info->non_3dfix_seen = true;
        }
    } else {
        if (_vehicle->time_since_boot_T() &&
            _vehicle->time_since_boot() < info->first_3D_fix_T) {
            // reboot; chance of getting another result:
            info->first_3D_fix_found = false;
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

void Analyzer_GPS_Fix::add_no_firstfixtime_result(const std::string result_key UNUSED,
                                                  FirstFixInfo *info UNUSED)
{
    Analyzer_GPS_No_FirstFixTime_Result *result =
        new Analyzer_GPS_No_FirstFixTime_Result();

    result->set_reason("No 3D fix was ever acquired");
    result->increase_severity_score(20);
    result->add_source(_data_sources.get(std::string("GPSINFO_FIXTYPE_") + result_key));
    add_result(result);
}

void Analyzer_GPS_Fix::close_result(const std::string result_key,
                                    Analyzer_GPS_Fix_Result *result)
{
    result->set_T_stop(_vehicle->T());

    result->add_evidence(string_format("satellites-visible: %d", result->satellites()));
    result->add_evidence(string_format("HDop: %.2f", result->hdop()));
    result->add_evidence(string_format("sAcc: %.2f", result->sacc()));
    result->add_evidence(string_format("satellites-visible-threshold: %d", satellites_visible_threshold()));
    result->add_evidence(string_format("HDop-threshold: %f", hdop_threshold()));
    result->add_evidence(string_format("sAcc-warn-threshold: %f", sacc_threshold_warn()));
    result->add_evidence(string_format("sAcc-fail-threshold: %f", sacc_threshold_fail()));

    if (result->sacc() > sacc_threshold_fail()) {
        result->increase_severity_score(20);
    } else if (result->sacc() > sacc_threshold_warn()) {
        result->increase_severity_score(10);
    }

    if (result->satellites() <= satellites_visible_threshold()) {
        result->increase_severity_score(10);
    }
    if (result->hdop() >= hdop_threshold()) {
        result->increase_severity_score(10);
    }

    result->add_source(_data_sources.get(std::string("GPSINFO_") + result->name()));
    add_result(result);
    _result.erase(result_key);
}

void Analyzer_GPS_Fix::update_result(AnalyzerVehicle::GPSInfo *gpsinfo)
{
    Analyzer_GPS_Fix_Result *result = _result[gpsinfo->name()];
    if (result->satellites() > gpsinfo->satellites()) {
        result->set_satellites(gpsinfo->satellites());
    }
    if (result->hdop() < gpsinfo->hdop()) {
        result->set_hdop(gpsinfo->hdop());
    }
    if (result->sacc() < gpsinfo->sacc()) {
        result->set_sacc(gpsinfo->sacc());
    }
}

void Analyzer_GPS_Fix::open_result(AnalyzerVehicle::GPSInfo *gpsinfo)
{
    Analyzer_GPS_Fix_Result *result = new Analyzer_GPS_Fix_Result(gpsinfo->name());
    result->set_T_start(_vehicle->T());

    result->set_status(analyzer_status_fail);
    result->set_reason("GPS Fix Is Bad");

    result->set_satellites(gpsinfo->satellites());
    result->set_hdop(gpsinfo->hdop());
    result->set_sacc(gpsinfo->sacc());
    _result[gpsinfo->name()] = result;
}


void Analyzer_GPS_Fix::close_results()
{
    auto next = _result.begin();
    while (next != _result.end()) {
        auto current = next;
        next++;
        if ((*current).second == nullptr) {
            // shouldn't happen
            continue;
        }
        close_result((*current).first, (*current).second);
    }
}


void Analyzer_GPS_Fix::close_result_ff(const std::string result_key, FirstFixInfo *info)
{
    if (!info->first_3D_fix_found) {
        add_no_firstfixtime_result(result_key, info);
    }
}

void Analyzer_GPS_Fix::close_results_ff()
{
    auto next = _result_ff.begin();
    while (next != _result_ff.end()) {
        auto current = next;
        next++;
        if ((*current).second == nullptr) {
            // shouldn't happen
            continue;
        }
        close_result_ff((*current).first, (*current).second);
    }
}
