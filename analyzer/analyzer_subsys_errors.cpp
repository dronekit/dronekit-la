#include "analyzer_subsys_errors.h"

#include "util.h"
#include "analyzer_util.h"

void Analyzer_Subsys_Errors_Result::to_json(Json::Value &root) const
{
    Analyzer_Result_Period::to_json(root);
    root["subsystem"] = _subsys;
    root["subsystem-string"] = _subsys_string;
    root["ecode"] = _ecode;
    root["ecode-string"] = _ecode_string;
}

bool Analyzer_Subsys_Errors::configure(INIReader *config) {
    if (!Analyzer::configure(config)) {
	return false;
    }
    return true;
}

void Analyzer_Subsys_Errors::end_of_log(uint32_t packet_count UNUSED)
{
    auto subsys_errors = _vehicle->subsys_errors();

    for (auto it = subsys_errors.begin(); it != subsys_errors.end(); ++it) {
        // ::fprintf(stderr, "would evaluate (%s)\n", (*it).first.c_str());
        uint32_t subsys = (*it).first;
        if (_results[subsys] != NULL) {
            close_result(subsys);
        }
    }

    // if (!have) {
    //     Analyzer_Result_Summary *summary = new Analyzer_Result_Summary();
    //     summary->set_status(analyzer_status_warn);
    //     summary->set_reason("Sensor health never updated");
    //     summary->add_source(_data_sources.get("SENSORS_HEALTH"));
    //     add_result(summary);
    // }

}

void Analyzer_Subsys_Errors::close_result(const uint32_t subsys)
{
    Analyzer_Subsys_Errors_Result *result = _results[subsys];

    result->set_T_stop(_vehicle->T());
    add_result(result);
    _results[subsys] = NULL;
}

const std::map<const uint32_t, const std::string> _subsys_name = {
    { ERROR_SUBSYSTEM_MAIN, "MAIN" },
    { ERROR_SUBSYSTEM_RADIO, "RADIO" },
    { ERROR_SUBSYSTEM_COMPASS, "COMPASS" },
    { ERROR_SUBSYSTEM_OPTFLOW, "OPTFLOW" },
    { ERROR_SUBSYSTEM_FAILSAFE_RADIO, "FAILSAFE_RADIO" },
    { ERROR_SUBSYSTEM_FAILSAFE_BATT, "FAILSAFE_BATT" },
    { ERROR_SUBSYSTEM_FAILSAFE_GPS, "FAILSAFE_GPS" },   // not used
    { ERROR_SUBSYSTEM_FAILSAFE_GCS, "FAILSAFE_GCS" },
    { ERROR_SUBSYSTEM_FAILSAFE_FENCE, "FAILSAFE_FENCE" },
    { ERROR_SUBSYSTEM_FLIGHT_MODE, "FLIGHT_MODE" },
    { ERROR_SUBSYSTEM_GPS, "GPS" },
    { ERROR_SUBSYSTEM_CRASH_CHECK, "CRASH_CHECK" },
    { ERROR_SUBSYSTEM_FLIP, "FLIP" },
    { ERROR_SUBSYSTEM_AUTOTUNE, "AUTOTUNE" },
    { ERROR_SUBSYSTEM_PARACHUTE, "PARACHUTE" },
    { ERROR_SUBSYSTEM_EKFCHECK, "EKFCHECK" },
    { ERROR_SUBSYSTEM_FAILSAFE_EKFINAV, "FAILSAFE_EKFINAV" },
    { ERROR_SUBSYSTEM_BARO, "BARO" },
    { ERROR_SUBSYSTEM_CPU, "CPU" },
    { ERROR_SUBSYSTEM_FAILSAFE_ADSB, "FAILSAFE_ADSB" },
    { ERROR_SUBSYSTEM_TERRAIN, "TERRAIN" },
    { ERROR_SUBSYSTEM_NAVIGATION, "NAVIGATION" },
    { ERROR_SUBSYSTEM_FAILSAFE_TERRAIN, "FAILSAFE_TERRAIN" },
    { ERROR_SUBSYSTEM_EKF_PRIMARY, "EKF_PRIMARY" },
};

/// @brief Map from subsystem id to error code values
/// @note Correct only for ArduCopter ATM
/// @note taken from ArduCopter/defines.h
const std::map<const uint32_t, std::map<const uint32_t, const std::string>> _subsys_errors_strings = {
    { ERROR_SUBSYSTEM_RADIO, {
            { ERROR_CODE_RADIO_LATE_FRAME, "Late Frame" },
        }
    },
    { ERROR_SUBSYSTEM_FAILSAFE_RADIO, {
            { ERROR_CODE_FAILSAFE_OCCURRED, "Occured" },
            { ERROR_CODE_FAILSAFE_RESOLVED, "Resolved" },
        }
    },
    { ERROR_SUBSYSTEM_FAILSAFE_BATT, {
            { ERROR_CODE_FAILSAFE_OCCURRED, "Occured" },
            { ERROR_CODE_FAILSAFE_RESOLVED, "Resolved" },
        }
    },
    { ERROR_SUBSYSTEM_FAILSAFE_GPS, {
            { ERROR_CODE_FAILSAFE_OCCURRED, "Occured" },
            { ERROR_CODE_FAILSAFE_RESOLVED, "Resolved" },
        }
    },
    { ERROR_SUBSYSTEM_FAILSAFE_GCS, {
            { ERROR_CODE_FAILSAFE_OCCURRED, "Occured" },
            { ERROR_CODE_FAILSAFE_RESOLVED, "Resolved" },
        }
    },
    { ERROR_SUBSYSTEM_FAILSAFE_FENCE, {
            { ERROR_CODE_FAILSAFE_OCCURRED, "Occured" },
            { ERROR_CODE_FAILSAFE_RESOLVED, "Resolved" },
        }
    },
    { ERROR_SUBSYSTEM_EKFCHECK, {
            { ERROR_CODE_EKFCHECK_BAD_VARIANCE, "Bad Variance" },
        }
    },
    { ERROR_SUBSYSTEM_BARO, {
            { ERROR_CODE_BARO_GLITCH, "Baro Glitch" },
        }
    },
    { ERROR_SUBSYSTEM_GPS, {
            { ERROR_CODE_GPS_GLITCH, "GPS Glitch" },
        }
    },
};

const std::string Analyzer_Subsys_Errors::subsys_string(const uint32_t subsys) const
{
    return _subsys_name.at(subsys);
}

const std::string Analyzer_Subsys_Errors::ecode_string(uint32_t subsys,
                                                       uint32_t ecode) const
{
    try {
        std::map<const uint32_t, const std::string> bob = _subsys_errors_strings.at(subsys);
        return bob.at(ecode);
    } catch (std::out_of_range&) {
    }
    switch(ecode) {
    case ERROR_CODE_ERROR_RESOLVED:
        return "Resolved";
    case ERROR_CODE_FAILED_TO_INITIALISE:
        return "Failed to initialise";
    case ERROR_CODE_UNHEALTHY:
        return "Unhealthy";
    default:
        return "Unknown";
    }
}

void Analyzer_Subsys_Errors::open_result(const uint32_t subsys, const uint32_t ecode)
{
    _results[subsys] = new Analyzer_Subsys_Errors_Result(subsys,
                                                         subsys_string(subsys),
                                                         ecode,
                                                         ecode_string(subsys, ecode));
    _results[subsys]->set_reason("A subsystem failure has occured");
    _results[subsys]->set_T_start(_vehicle->T());
    _results[subsys]->set_status(analyzer_status_fail);
    _results[subsys]->add_source(_data_sources.get("SUBSYS_ERRORS"));
    _results[subsys]->increase_severity_score(10);
}


void Analyzer_Subsys_Errors::evaluate()
{
    auto subsys_errors = _vehicle->subsys_errors();

    for (auto it = subsys_errors.begin(); it != subsys_errors.end(); ++it) {
        uint32_t subsys = (*it).first;
        uint32_t new_ecode = (*it).second;
        // ::fprintf(stderr, "would evaluate (%s)\n", name.c_str());
        if (_results[subsys] != NULL) {
            if (_results[subsys]->ecode() != new_ecode) {
                close_result(subsys);
            }
        }
        if (_results[subsys] == NULL && new_ecode) {
            open_result(subsys, new_ecode);
        }
    }
}

