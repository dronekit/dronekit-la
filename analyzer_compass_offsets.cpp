#include "analyzer_compass_offsets.h"

#include <stdio.h>

#include "util.h"
#include "analyzer_util.h"

void Analyzer_Compass_Offsets_Result::to_json(Json::Value &root) const {
    Analyzer_Result_Event::to_json(root);
}

bool Analyzer_Compass_Offsets::configure(INIReader *config) {
    return true;
}

// true if we have all elements of vector and we haven't processed
// this vector before
bool Analyzer_Compass_Offsets::new_compass_results()
{
    if (!_vehicle->param_seen("COMPASS_OFS_X") ||
        !_vehicle->param_seen("COMPASS_OFS_Y") ||
        !_vehicle->param_seen("COMPASS_OFS_Z")) {
        // we haven't seen the entire vector yet
        return false;
    }

    if (_vehicle->param_modtime("COMPASS_OFS_X") == modtime_compass_ofs[0] &&
        _vehicle->param_modtime("COMPASS_OFS_Y") == modtime_compass_ofs[1] &&
        _vehicle->param_modtime("COMPASS_OFS_Z") == modtime_compass_ofs[2]) {
        // nothing has changed; at time of writing we get called for
        // every parameter which is set (low-bandwidth, so *shrug*)
        return false;
    }

    modtime_compass_ofs[0] = _vehicle->param_modtime("COMPASS_OFS_X");
    modtime_compass_ofs[1] = _vehicle->param_modtime("COMPASS_OFS_Y");
    modtime_compass_ofs[2] = _vehicle->param_modtime("COMPASS_OFS_Z");

    return true;
}

void Analyzer_Compass_Offsets::evaluate()
{
    if (result_count() >= MAX_COMPASS_OFFSET_RESULTS) {
        compass_offset_results_overrun = true;
        return;
    }

    if (! new_compass_results()) {
        return;
    }

    Analyzer_Compass_Offsets_Result *result = new Analyzer_Compass_Offsets_Result();
    result->set_T(_vehicle->T());
    result->set_lens(_vehicle->param("COMPASS_OFS_X"),
                     _vehicle->param("COMPASS_OFS_Y"),
                     _vehicle->param("COMPASS_OFS_Z"));
    result->add_evidence(string_format("COMPASS_OFS_X=%f", result->lens()[0]));
    result->add_evidence(string_format("COMPASS_OFS_Y=%f", result->lens()[1]));
    result->add_evidence(string_format("COMPASS_OFS_Z=%f", result->lens()[2]));

    result->add_series(_data_sources.get("PARAM"));

    double len = result->lens().len();
    if (len >= fail_offset) {
        result->set_status(analyzer_status_fail);
        result->add_evilness(5);
        result->set_reason("Compass offsets in parameters are out of bounds");
        result->add_evidence
            (string_format("COMPASS_OFS %f >= %f", len, fail_offset));
    } else if (len >= warn_offset) {
        result->set_status(analyzer_status_warn);
        result->add_evilness(2);
        result->set_reason("Compass offsets in parameters are out of bounds");
        result->add_evidence
            (string_format("COMPASS_OFS %f >= %f", len, warn_offset));
    } else if (is_zero(len)) {
        result->set_status(analyzer_status_warn);
        result->add_evilness(4);
        result->set_reason("Compass offsets in parameters are zero");
        result->add_evidence
            (string_format("COMPASS_OFS %f == 0", len));
    } else {
        result->set_status(analyzer_status_ok);
        result->add_evidence
            (string_format("COMPASS_OFS %f < %f", len, warn_offset));
    }

    add_result(result);
}

void Analyzer_Compass_Offsets::end_of_log(uint32_t packet_count)
{
    if (! _vehicle->param_seen("COMPASS_OFS_X")) {
        Analyzer_Compass_Offsets_Result *result = new Analyzer_Compass_Offsets_Result();
        result->set_status(analyzer_status_fail);
        result->add_series(_data_sources.get("PARAM"));
        result->set_reason("No compass offset parameter set seen");
        result->add_evilness(5);
        add_result(result);
    }
}
