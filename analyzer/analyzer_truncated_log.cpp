#include "analyzer_truncated_log.h"

#include "util.h"
#include "analyzer_util.h"

bool Analyzer_Truncated_Log::configure(INIReader *config)
{
    if (!Analyzer::configure(config)) {
	return false;
    }

    max_last_relative_altitude = config->GetReal("loganalyzer", "truncated_log::max_last_relative_altitude", 15.0f);
    _min_low_voltage = config->GetReal("loganalyzer", "truncated_log::low_voltage", 4.6f);
    _max_voltage_delta = config->GetReal("loganalyzer", "truncated_log::max_voltage_delta", 4.6f);

    return true;
}

void Analyzer_Truncated_Log::evaluate()
{
    bool new_is_flying = _vehicle->is_flying();
    if (new_is_flying && !_old_is_flying) {
        _result.set_takeoff_altitude(_vehicle->altitude());
        _old_is_flying = new_is_flying;
    } else if (!new_is_flying && _old_is_flying) {
        _old_is_flying = new_is_flying;
    }
    double vcc  =_vehicle->autopilot().vcc();
    if (_vehicle->autopilot().vcc_T()) {
        if (vcc > _highest_voltage) {
            _highest_voltage = vcc;
        }
        if (vcc < _lowest_voltage) {
            _lowest_voltage = vcc;
        }
    }
}

void Analyzer_Truncated_Log::end_of_log(const uint32_t packet_count UNUSED)
{
    double last_altitude = _vehicle->altitude();
    _result.set_last_altitude(last_altitude);

    if (_vehicle->alt_modtime() > 0) {
        _result.add_evidence(string_format("Final altitude %f metres", _result.last_altitude()));
        if (_vehicle->is_flying()) {

            _result.add_evidence(string_format("Takeoff altitude %f metres", _result.takeoff_altitude()));
            double last_relative_altitude = _result.last_altitude() - _result.takeoff_altitude();
            _result.add_evidence(string_format("Final relative altitude %f metres", last_relative_altitude));

            _result.set_status(analyzer_status_fail);
            _result.increase_severity_score(10);
            if (last_relative_altitude > max_last_relative_altitude) {
                _result.increase_severity_score(10);
            }
            _result.add_source(_data_sources.get("SERVO_OUTPUT"));
            _result.add_source(_data_sources.get("ALTITUDE"));
            _result.set_reason("Log ended while craft still flying");
            _result.add_evidence("Vehicle still flying");

            _result.add_source(_data_sources.get("AUTOPILOT_VCC"));
            if (_vehicle->autopilot().vcc_T()) {
                if (_lowest_voltage < _min_low_voltage) {
                    _result.add_evidence(string_format("Low AutoPilot Voltage (%0.2f < %0.2f", _lowest_voltage, _min_low_voltage));
                } else {
                    _result.add_evidence(string_format("AutoPilot Voltage OK (%0.2f > %0.2f)", _lowest_voltage, _min_low_voltage));
                }
                double voltage_delta = _highest_voltage - _lowest_voltage;
                if (voltage_delta > _max_voltage_delta) {
                    _result.add_evidence(string_format("AutoPilot Voltage Delta BAD (%0.2f > %0.2f)", voltage_delta, _max_voltage_delta));
                }
            }
        } else {
            _result.set_status(analyzer_status_ok);
            _result.add_source(_data_sources.get("SERVO_OUTPUT"));
            _result.add_source(_data_sources.get("ALTITUDE"));
            _result.set_reason("Log truncation not detected");
        }
    } else {
        _result.set_status(analyzer_status_warn);
        _result.increase_severity_score(5);
        _result.set_reason("Altitude never changed");
        _result.add_source(_data_sources.get("ALTITUDE"));
    }

    add_result(&_result);
}
