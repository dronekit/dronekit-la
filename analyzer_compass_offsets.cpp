#include "analyzer_compass_offsets.h"

#include <stdio.h>

#include "util.h"
#include "analyzer_util.h"

bool Analyzer_Compass_Offsets::configure(INIReader *config) {
    return true;
}

// true if we have all elements of vector and we haven't processed
// this vector before
bool Analyzer_Compass_Offsets::new_compass_results()
{
    std::string p_x = "COMPASS_OFS" + _param_extra_string + "_X";
    std::string p_y = "COMPASS_OFS" + _param_extra_string + "_Y";
    std::string p_z = "COMPASS_OFS" + _param_extra_string + "_Z";

    if (!_vehicle->param_seen(p_x) ||
        !_vehicle->param_seen(p_y) ||
        !_vehicle->param_seen(p_z)) {
        // we haven't seen the entire vector yet
        return false;
    }

    if (_vehicle->param_modtime(p_x) == modtime_compass_ofs[0] &&
        _vehicle->param_modtime(p_y) == modtime_compass_ofs[1] &&
        _vehicle->param_modtime(p_z) == modtime_compass_ofs[2]) {
        // nothing has changed; at time of writing we get called for
        // every parameter which is set (low-bandwidth, so *shrug*)
        return false;
    }

    modtime_compass_ofs[0] = _vehicle->param_modtime(p_x);
    modtime_compass_ofs[1] = _vehicle->param_modtime(p_y);
    modtime_compass_ofs[2] = _vehicle->param_modtime(p_z);

    return true;
}

bool Analyzer_Compass_Offsets::compass_use(const std::string param_extra_string)
{
    if (! _vehicle->param_seen("COMPASS_USE" + _param_extra_string)) {
        return false;
    }
    if (is_zero(_vehicle->param("COMPASS_USE" + _param_extra_string))) {
        // told not to use this compass, so ignore it
        return false;
    }
    return true;
}

void Analyzer_Compass_Offsets::evaluate()
{
    if (result_count() >= MAX_COMPASS_OFFSET_RESULTS) {
        compass_offset_results_overrun = true;
        return;
    }

    bool _compass_use = compass_use(_param_extra_string);
    bool _new_compass_results = new_compass_results();
    if (! _compass_use) {
        _old_compass_use = false;
        return;
    }
    if (_old_compass_use && !_new_compass_results) {
        return;
    }
    _old_compass_use = true;

    std::string p_x = "COMPASS_OFS" + _param_extra_string + "_X";
    std::string p_y = "COMPASS_OFS" + _param_extra_string + "_Y";
    std::string p_z = "COMPASS_OFS" + _param_extra_string + "_Z";

    Analyzer_Compass_Offsets_Result *result = new Analyzer_Compass_Offsets_Result();
    result->set_T(_vehicle->T());
    result->set_lens(_vehicle->param(p_x),
                     _vehicle->param(p_y),
                     _vehicle->param(p_z));
    result->add_evidence(string_format(p_x + "=%f", result->lens()[0]));
    result->add_evidence(string_format(p_y + "=%f", result->lens()[1]));
    result->add_evidence(string_format(p_z + "=%f", result->lens()[2]));

    result->add_source(_data_sources.get("PARAM"));

    double len = result->lens().len();
    if (len >= fail_offset) {
        result->set_status(analyzer_status_fail);
        result->add_evilness(5);
        result->set_reason("Compass offsets in parameters are out of bounds");
        result->add_evidence
            (string_format("COMPASS_OFS" + _param_extra_string + " %f >= %f", len, fail_offset));
    } else if (len >= warn_offset) {
        result->set_status(analyzer_status_warn);
        result->add_evilness(2);
        result->set_reason("Compass offsets in parameters are out of bounds");
        result->add_evidence
            (string_format("COMPASS_OFS" + _param_extra_string + " %f >= %f", len, warn_offset));
    } else if (is_zero(len)) {
        result->set_status(analyzer_status_warn);
        result->add_evilness(4);
        result->set_reason("Compass offsets in parameters are zero");
        result->add_evidence
            (string_format("COMPASS_OFS" + _param_extra_string + " %f == 0", len));
    } else {
        result->set_status(analyzer_status_ok);
        result->add_evidence
            (string_format("COMPASS_OFS" + _param_extra_string + " %f < %f", len, warn_offset));
    }

    add_result(result);
}

void Analyzer_Compass_Offsets::end_of_log(uint32_t packet_count)
{
    if (! compass_use(_param_extra_string)) {
        // note that this means that if a compass gets turned off
        // half-way through the log then we won't get any results...
        return;
    }
    if (! _vehicle->param_seen("COMPASS_OFS" + _param_extra_string + "_X")) {
        Analyzer_Compass_Offsets_Result *result = new Analyzer_Compass_Offsets_Result();
        result->set_status(analyzer_status_fail);
        result->add_source(_data_sources.get("PARAM"));
        result->set_reason("No compass offset parameter set seen");
        result->add_evilness(5);
        add_result(result);
    }
}
