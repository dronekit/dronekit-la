#include "analyzer_arming_checks.h"

#include "util.h"
#include "analyzer_util.h"

void Analyzer_Arming_Checks::evaluate()
{
    if (!_vehicle->param_seen("ARMING_CHECK")) {
        return;
    }

    if (_armed == _vehicle->is_armed()) {
        // we have not changed state since last we checked
        return;
    }
    _armed = _vehicle->is_armed();

    if (! _vehicle->is_armed()) {
        // not interested in armed -> disarmed transitions
        return;
    }
    // we have armed since last we checked
    _armed = true;
    
    uint32_t arming_check = (uint32_t)_vehicle->param("ARMING_CHECK");
    if (arming_check & ARMING_CHECK_ALL) {
        // all is right with the world
        return;
    }

    for (std::map<uint32_t, const char *>::const_iterator it = _bit_to_name.begin(); it != _bit_to_name.end(); ++it) {
        if ((*it).first == ARMING_CHECK_ALL ||
            (*it).first == ARMING_CHECK_NONE) {
            continue;
        }
        if (!(arming_check & (*it).first)) {
            Analyzer_Arming_Checks_Result *result = new
                Analyzer_Arming_Checks_Result();
            result->set_status(analyzer_status_fail);
            result->set_reason("Some of the arming checks were disabled when the craft was  armed");
            result->add_evidence(string_format("Arming flags: %u", arming_check));
            result->add_source(_data_sources.get("PARAM"));
            result->add_source(_data_sources.get("ARMING"));

            result->set_T(_vehicle->T());
            result->set_arming_check(arming_check);
            result->increase_severity_score(10);
            for (std::map<uint32_t, const char *>::const_iterator it = _bit_to_name.begin(); it != _bit_to_name.end(); ++it) {
                if (!(result->arming_check() & (*it).first)) {
                    result->add_evidence(string_format("%s check disabled", (*it).second));
                    result->increase_severity_score(1);
                }
            }
            add_result(result);
            break;
        }
    }
}
