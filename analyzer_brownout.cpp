#include "analyzer_brownout.h"

#include "util.h"
#include "analyzer_util.h"

bool Analyzer_Brownout::configure(INIReader *config)
{
    if (!Analyzer::configure(config)) {
	return false;
    }

    _result.set_status(analyzer_status_ok);
    _result.set_reason("No brownout detected");
    
    return true;
}

void Analyzer_Brownout::evaluate()
{
    bool new_is_flying = _vehicle->is_flying();
    if (new_is_flying && !_old_is_flying) {
        _result.set_takeoff_altitude(_vehicle->alt());
        _old_is_flying = new_is_flying;
    } else if (!new_is_flying && _old_is_flying) {
        _old_is_flying = new_is_flying;
    }
}

void Analyzer_Brownout::end_of_log(const uint32_t packet_count)
{
    double last_altitude = _vehicle->alt();
    _result.set_last_altitude(last_altitude);

    if (_vehicle->alt_modtime() > 0) {
        _result.add_evidence(string_format("Final altitude %f metres", _result.last_altitude()));
        if (_vehicle->is_flying() &&
            last_altitude > _result.takeoff_altitude() + max_last_relative_altitude) {
            _result.add_evidence(string_format("Takeoff altitude %f metres", _result.takeoff_altitude()));
            _result.set_status(analyzer_status_fail);
            _result.add_evilness(10);
            _result.add_series(_data_sources.get("SERVO_OUTPUT"));
            _result.add_series(_data_sources.get("ALTITUDE"));
            _result.set_reason("Log ended while craft still flying");
            _result.add_evidence("Vehicle still flying");
        }
    } else {
        _result.set_status(analyzer_status_warn);
        _result.add_evilness(5);
        _result.set_reason("Altitude never changed");
        _result.add_series(_data_sources.get("ALTITUDE"));
    }

    add_result(&_result);
}
