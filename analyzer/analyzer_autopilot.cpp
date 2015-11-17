#include "analyzer_autopilot.h"

#include "util.h"
#include "analyzer_util.h"

bool Analyzer_Autopilot::configure(INIReader *config) {
    if (!Analyzer::configure(config)) {
	return false;
    }
    return true;
}

void Analyzer_Autopilot::end_of_log(uint32_t packet_count UNUSED)
{
}

void Analyzer_Autopilot::close_result_slices_max()
{
    _result_slices_max->add_evidence(string_format("slices-max-threshold=%u", _result_slices_max->_slices_max_max));
    _result_slices_max->set_T_stop(_vehicle->T());
    add_result(_result_slices_max);
    _result_slices_max = NULL;
}

void Analyzer_Autopilot::open_result_slices_max(uint16_t slices)
{
    _result_slices_max = new Analyzer_AutoPilot_Result_Slices_Max();
    _result_slices_max->set_T_start(_vehicle->T()); 
    _result_slices_max->add_source(_data_sources.get("AUTOPILOT_SCHEDULING"));
    _result_slices_max->set_reason("Severe scheduler overruns");
    _result_slices_max->add_evidence(string_format("slices-threshold=%d", _slices_max_threshold));
    _result_slices_max->set_status(analyzer_status_fail);
    _result_slices_max->add_evilness(10);
    update_result_slices_max(slices);
}

void Analyzer_Autopilot::update_result_slices_max(const uint16_t slices)
{
    if (slices > _result_slices_max->_slices_max_max) {
        _result_slices_max->_slices_max_max = slices;
    }
}

void Analyzer_Autopilot::evaluate()
{
    if (_vehicle->autopilot().overruns_T()) {
        // bool overruns_bad = (100 * _vehicle->autopilot_overruns() / _vehicle->autopilot_overruns_T()) >= 1.0f;             // more than one percent overruns
        count_overruns += _vehicle->autopilot().overruns();
        count_loops += _vehicle->autopilot().loopcount();
    }

    if (_vehicle->autopilot().slices_max_T()) {
        auto slices_max = _vehicle->autopilot().slices_max();
        bool bad = slices_max >= _slices_max_threshold;
        if (_result_slices_max != NULL) {
            if (bad) {
                update_result_slices_max(slices_max);
            } else {
                close_result_slices_max();
            }
        } else if (bad) {
            open_result_slices_max(slices_max);
        }
    }
}
