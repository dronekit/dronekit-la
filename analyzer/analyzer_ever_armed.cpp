#include "analyzer_ever_armed.h"

#include "util.h"
#include "analyzer_util.h"

void Analyzer_Ever_Armed_Result::to_json(Json::Value &root) const
{
    Analyzer_Result_Summary::to_json(root);
    root["timestamp-first-armed"] = (Json::UInt64)arm_time();
}

bool Analyzer_Ever_Armed::configure(INIReader *config)
{
    if (!Analyzer::configure(config)) {
	return false;
    }

    _result.set_status(analyzer_status_fail);
    _result.set_reason("The vehicle never armed");
    
    return true;
}

void Analyzer_Ever_Armed::evaluate()
{
    if (_result.status() == analyzer_status_ok) {
        return;
    }

    if (_vehicle->is_armed()) {
        _result.set_arm_time(_vehicle->T());
        _result.set_status(analyzer_status_ok);
        _result.set_reason("The vehicle armed");
        _result.add_evidence(string_format("Armed at %lu", _result.arm_time()));
    }
}

void Analyzer_Ever_Armed::end_of_log(const uint32_t packet_count UNUSED)
{
    _result.add_source(_data_sources.get("ARMING"));
    add_result(&_result);
}
