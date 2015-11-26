#include "analyzer_battery.h"

#include "util.h"
#include "analyzer_util.h"

bool Analyzer_Battery::configure(INIReader *config) {
    if (!Analyzer::configure(config)) {
        return false;
    }
    low_battery_threshold = config->GetReal("loganalyzer", "battery::low_battery_threshold", 15.0f);
    return true;
}

void Analyzer_Battery::evaluate()
{
    if (_vehicle->battery_remaining_T() &&
        _vehicle->battery_remaining() < lowest_battery_remaining_seen) {
        lowest_battery_remaining_seen = _vehicle->battery_remaining();
        lowest_battery_remaining_seen_T = _vehicle->T();
    }
    if (_vehicle->battery_in_failsafe()) {
        seen_failsafe_battery_event = true;
        seen_failsafe_battery_event_T = _vehicle->battery_in_failsafe_T();
    }
}

/* FIXME: this result should be opened as soon as the event happens! */
void Analyzer_Battery::end_of_log(const uint32_t packet_count UNUSED)
{
    if (lowest_battery_remaining_seen_T) {
        result->add_evidence(string_format("battery-remaining=%f%%", lowest_battery_remaining_seen));
        result->add_evidence(string_format("failsafe=%f%%", low_battery_threshold));
        if (lowest_battery_remaining_seen < low_battery_threshold) {
            result->set_status(analyzer_status_fail);
            result->set_reason("Battery fell below failsafe threshold");
            result->add_evidence(string_format("Battery below failsafe (%.0f%% < %.0f%%)",
                                               lowest_battery_remaining_seen, low_battery_threshold));
            result->increase_severity_score(20);
        } else {
            result->set_status(analyzer_status_ok);
            result->set_reason("Battery never below failsafe");
        }
        add_result(result);
    }

    if (seen_failsafe_battery_event_T) {
        result->set_status(analyzer_status_fail);
        result->set_reason("Battery failsafe event received");
        result->add_evidence(string_format("Failsafe set at %u",
                                           seen_failsafe_battery_event_T));
        result->add_source(_data_sources.get("BATTERY_FAILSAFE"));
        result->increase_severity_score(20);
        add_result(result);
    }
}
