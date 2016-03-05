#include "analyzer_fs_batt.h"

#include "util.h"
#include "analyzer_util.h"

bool Analyzer_FS_Batt::configure(INIReader *config) {
    if (!Analyzer::configure(config)) {
        return false;
    }
//    low_fs_batt_threshold = config->GetReal("loganalyzer", "fs_batt::low_fs_batt_threshold", 15.0f);
    return true;
}

void Analyzer_FS_Batt::open_result()
{
    double expected_voltage = _vehicle->param("FS_BATT_VOLTAGE");
    double actual_voltage = _vehicle->battery_voltage();

    _result = new Analyzer_FS_Batt_Result();

    _result->add_source(_data_sources.get("PARAM"));
    _result->add_source(_data_sources.get("BATTERY_VOLTAGE"));
    _result->set_T_start(_vehicle->T());
    _result->set_reason("Parameter FS_BATT_VOLTAGE differs from actual voltage");
    _result->add_evidence(string_format("EXPECTED=%f",expected_voltage));
    _result->add_evidence(string_format("ACTUAL=%f",actual_voltage));

    _result->set_status(analyzer_status_fail);
    _result->set_severity_score(20);
    _result->set_delta_max(fabs(expected_voltage - actual_voltage));
    _result->set_fs_batt_voltage(_vehicle->param("FS_BATT_VOLTAGE"));
}

void Analyzer_FS_Batt::update_result()
{
    double expected_voltage = _vehicle->param("FS_BATT_VOLTAGE");
    double actual_voltage = _vehicle->battery_voltage();
    double delta = fabs(expected_voltage - actual_voltage);
    if (delta > _result->delta_max()) {
        _result->set_delta_max(delta);
    }
}
void Analyzer_FS_Batt::close_result()
{
    _result->set_T_stop(_vehicle->T());
    add_result(_result);
    _result = NULL;
}

bool Analyzer_FS_Batt::fs_batt_bad()
{
    if (_vehicle->param_seen("FS_BATT_ENABLE") &&
        is_zero(_vehicle->param("FS_BATT_ENABLE"))) {
        // FS not enabled, so who cares?
        return false;
    }

    double expected_voltage = _vehicle->param("FS_BATT_VOLTAGE");
    if (!_vehicle->param_seen("FS_BATT_ENABLE") &&
        is_zero(expected_voltage)) {
        return false;
    }

    // we don't really care if the actual voltage is *just* under the
    // expected voltage.  We expect the autopilot to trigger a
    // failsafe, and we check for that elsewhere.  What we do care
    // about is the failsafe voltage being with too far over the
    // actual voltage - this can happen if a UAV is e.g. configured
    // for 4S then at a later date run on 3S
    double actual_voltage = _vehicle->battery_voltage();
    double delta= actual_voltage - expected_voltage;
    // so this is for LiPo; guess the number of cells allow 0.5V...
    double allowed_delta = (expected_voltage / 3.5) * 0.7;
    // fprintf(stderr, "Variability allowed: %f (have %f)\n", allowed_delta, delta);
    if (delta < 0 || delta > allowed_delta) {
        return true;
    }

    return false;
}

void Analyzer_FS_Batt::evaluate()
{
    if (!_vehicle->param_seen("FS_BATT_VOLTAGE") ||
        _vehicle->battery_voltage_T() == 0) {
        return;
    }
    if (_vehicle->param_seen("FS_BATT_ENABLE") &&
        is_zero(_vehicle->param("FS_BATT_ENABLE"))) {
        // ArduCopter has this extra enable parameter (really a "failsafe action" enum)
        return;
    }

    bool bad = fs_batt_bad();
    if (bad) {
        if (_result == NULL) {
            // already underway
            open_result();
        } else if (! is_equal(_vehicle->param("FS_BATT_VOLTAGE"),
                              _result->fs_batt_voltage())) {
            close_result();
            open_result();
        }
        update_result();
    } else {
        if (_result != NULL) {
            close_result();
        }
    }
}

void Analyzer_FS_Batt::end_of_log(const uint32_t packet_count UNUSED)
{
    if (_result) {
        close_result();
    } else {
        if ((!_vehicle->param_seen("FS_BATT_VOLTAGE") ||
             _vehicle->battery_voltage_T() == 0) ||
            (!_vehicle->param_seen("FS_BATT_ENABLE"))) {
            // we never even really got started...
            Analyzer_Result_Summary *summary = new Analyzer_Result_Summary();
            summary->set_status(analyzer_status_warn);
            summary->set_severity_score(20);
            summary->set_reason("No battery failsafe available");
            summary->add_source(_data_sources.get("PARAM"));
            summary->add_source(_data_sources.get("BATTERY_VOLTAGE"));
            if (!_vehicle->param_seen("FS_BATT_VOLTAGE")) {
                summary->add_evidence("FS_BATT_VOLTAGE parameter never seen");
            }
            if (_vehicle->battery_voltage_T() == 0) {
                summary->add_evidence("Battery voltage never updated");
            }
            if (!_vehicle->param_seen("FS_BATT_ENABLE") &&
                _vehicle->battery_voltage_T() != 0 &&
                _vehicle->param_seen("FS_BATT_VOLTAGE") &&
                is_zero(_vehicle->param("FS_BATT_VOLTAGE"))) {
                summary->add_evidence("FS_BATT_VOLTAGE is zero");
            }
            add_result(summary);
        }
    }
}
