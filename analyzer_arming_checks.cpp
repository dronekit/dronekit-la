#include "analyzer_arming_checks.h"

#include "util.h"
#include "analyzer_util.h"

void Analyzer_Arming_Checks::evaluate()
{
    if (!_vehicle->param_seen("ARMING_CHECK")) {
        return;
    }

    if (_armed != _vehicle->is_armed()) {
        // we have changed state since last we checked
        _armed = _vehicle->is_armed();

        if (_vehicle->is_armed()) {
            // we have armed since last we checked
            _armed = true;
            uint32_t arming_check = (uint32_t)_vehicle->param("ARMING_CHECK");
            if (arming_check & ARMING_CHECK_ALL) {
                // all is right with the world
            } else {
                for (std::map<uint32_t, const char *>::const_iterator it = _bit_to_name.begin(); it != _bit_to_name.end(); ++it) {
                    if ((*it).first == ARMING_CHECK_ALL ||
                        (*it).first == ARMING_CHECK_NONE) {
                        continue;
                    }
                    if (!(arming_check & (*it).first)) {
                        results[next_result].T = _vehicle->T();
                        results[next_result].arming_check = arming_check;
                        next_result++;
                        break;
                    }
                }
            }            
        }
    }
}

void Analyzer_Arming_Checks::results_json_results_do_result(Json::Value &root,
                                                            const struct result result)
{
    root["timestamp"] = (Json::UInt64)(result.T);
    root["status"] = "FAIL";
    root["reason"] = "Some of the arming checks were disabled when the craft was  armed";

    Json::Value evidence(Json::arrayValue);
    evidence.append(string_format("Arming flags: %u", result.arming_check));
    uint32_t evilness = 10;
    for (std::map<uint32_t, const char *>::const_iterator it = _bit_to_name.begin(); it != _bit_to_name.end(); ++it) {
        if ((*it).first == ARMING_CHECK_ALL ||
            (*it).first == ARMING_CHECK_NONE) {
            continue;
        }
        if (!(result.arming_check & (*it).first)) {
            evidence.append(string_format("%s check disabled", (*it).second));
            evilness++;
        }
    }

    root["evilness"] = evilness;

    root["evidence"] = evidence;
    Json::Value series(Json::arrayValue);
    series.append("PARM");
    series.append("HEARTBEAT");
    root["series"] = series;
}
                                                          
void Analyzer_Arming_Checks::results_json_results(Json::Value &root)
{
    // if (!_vehicle->param_modtime("ARMING_CHECK")) {
    //     Json::Value result(Json::objectValue);
    //     root["timestamp"] = (Json::UInt64)T;
    //     root["status"] = "WARN";
    //     root["reason"] = "The ARMING_CHECK parameter was never set";
    //     result["evilness"] = 10;
    //     root.append(result);
    // } else {
        for (uint8_t i=0; i < next_result; i++) {
            Json::Value result(Json::objectValue);
            results_json_results_do_result(result, results[i]);
            root.append(result);
        }
    // }
}
