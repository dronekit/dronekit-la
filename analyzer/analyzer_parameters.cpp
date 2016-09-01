#include "analyzer_parameters.h"

#include "util.h"

#include <map>

void Analyzer_Parameters::open_result_trivial(const std::string name, const double value, const Param_Constraint_Trivial &constraints)
{
    Analyzer_Parameters_Result *result = new Analyzer_Parameters_Result();
    _open_results[name] = result;

    result->set_T_start(_vehicle->T());

    result->add_source(_data_sources.get("PARAM"));

    result->set_value(value);
    result->set_status(analyzer_status_fail);
    result->set_reason(constraints.description());
    result->add_evidence(string_format("name=%s", name.c_str()));
    result->add_evidence(string_format("value=%f", value));
    if (!constraints.min_ok(value)) {
        result->add_evidence("Parameter below minimum value");
        result->add_evidence(string_format("min=%f", constraints.min()));
    }
    if (!constraints.max_ok(value)) {
        result->add_evidence("Parameter above maximum value");
        result->add_evidence(string_format("max=%f", constraints.max()));
    }
    if (!constraints.default_ok(value)) {
        result->add_evidence("Parameter set to (bad) default value");
        result->add_evidence(string_format("default=%f", constraints.bad_default()));
    }

    add_result(result);
}

void Analyzer_Parameters::update_result_trivial(const std::string name, const double value, const Param_Constraint_Trivial &constraints)
{
    Analyzer_Parameters_Result *result = _open_results[name];
    double oldvalue = result->value();
    if (!is_equal(value, oldvalue)) {
        close_result_trivial(name);
        open_result_trivial(name, value, constraints);
    }
}
void Analyzer_Parameters::close_result_trivial(const std::string name)
{
    if (_open_results[name] != NULL) { // FIXME: this is a hack
        _open_results[name]->set_T_stop(_vehicle->T());
        _open_results.erase(name);
    }
}


void Analyzer_Parameters::evaluate_trivial()
{
    auto next = trivial_constraints.begin();
    while (next != trivial_constraints.end()) {
        auto current = next;
        next++;
        const std::string name = current->first;
        const Param_Constraint_Trivial &constraints = current->second;
        float value;

        if (name == "ANGLE_MAX" && !_enable_ANGLE_MAX_check) {
            continue;
        }
        if (_vehicle->param(name, value)) {
            if (constraints.value_ok(value)) {
                if (_open_results[name]) {
                    // parameter now set to good value...
                    close_result_trivial(name);
                }
            } else { // parameter value is bad...
                if (_open_results[name] == NULL) {
                    open_result_trivial(name, value, constraints);
                }
                update_result_trivial(name, value, constraints);
            }
        }
    }
}

// make sure someone running a PixHawk hasn't just copied parameters
// from an APM:
void Analyzer_Parameters::evaluate_log_bitmask()
{
    float log_bitmask;
    const AnalyzerVehicle::AutoPilot::AutoPilotHardware hw = _vehicle->autopilot().hardware();
    if (_vehicle->param("LOG_BITMASK", log_bitmask) &&
        hw != AnalyzerVehicle::AutoPilot::AutoPilotHardware::UNKNOWN) {
        bool bad = (hw == AnalyzerVehicle::AutoPilot::AutoPilotHardware::PX4V2 &&
                    is_equal(log_bitmask, log_bitmask_apm_default));
        if (bad) {
            if (_log_bitmask_bad_apm == nullptr) {
                _log_bitmask_bad_apm = new Analyzer_Parameters_Result();
                _log_bitmask_bad_apm->set_T_start(_vehicle->T());

                _log_bitmask_bad_apm->add_source(_data_sources.get("PARAM"));

                _log_bitmask_bad_apm->set_status(analyzer_status_fail);
                _log_bitmask_bad_apm->set_reason("AutoPilot is PixHawk hardware, log bitmask is APM default");
                _log_bitmask_bad_apm->add_evidence("autopilot=PX4v2");
                _log_bitmask_bad_apm->add_evidence(string_format("LOG_BITMASK=%f", log_bitmask));

                add_result(_log_bitmask_bad_apm);
            }
        } else {
            if (_log_bitmask_bad_apm != nullptr) {
                _log_bitmask_bad_apm->set_T_stop(_vehicle->T());
                _log_bitmask_bad_apm = nullptr;
            }
        }
    }
}


void Analyzer_Parameters::open_result_rc_channel(const std::string name, const uint16_t min, const uint16_t max, const uint16_t trim)
{
    Analyzer_Parameters_Result_RC *result = new Analyzer_Parameters_Result_RC();
    _open_results_rc_channels[name] = result;

    result->set_T_start(_vehicle->T());

    result->add_source(_data_sources.get("PARAM"));

    result->set_min(min);
    result->set_max(max);
    result->set_trim(trim);
    result->set_status(analyzer_status_fail);
    result->add_evidence(string_format("name=%s", name.c_str()));
    result->set_reason(string_format("%s parameters do not make sense", name.c_str()));
    ::fprintf(stderr, "min=%u max=%u trim=%u\n", min, max, trim);
    if (min > max) {
        result->add_evidence("Minimum greater than maximum");
        result->add_evidence(string_format("min=%u", min));
        result->add_evidence(string_format("max=%u", max));
    }
    if (trim < min) {
        result->add_evidence("Trim less than minimum");
        result->add_evidence(string_format("min=%u", min));
        result->add_evidence(string_format("trim=%u", trim));
    }
    if (trim > max) {
        result->add_evidence("Trim greater than maximum");
        result->add_evidence(string_format("max=%u", max));
        result->add_evidence(string_format("trim=%u", trim));
    }

    add_result(result);
}

void Analyzer_Parameters::update_result_rc_channel(const std::string name, const uint16_t min, const uint16_t max, const uint16_t trim)
{
    Analyzer_Parameters_Result_RC *result = _open_results_rc_channels[name];
    if (min != result->min() ||
        max != result->max() ||
        trim != result->trim()) {
        close_result_rc_channel(name);
        open_result_rc_channel(name, min, max, trim);
    }
}
void Analyzer_Parameters::close_result_rc_channel(const std::string name)
{
    if (_open_results_rc_channels[name] != NULL) { // FIXME: this is a hack
        _open_results_rc_channels[name]->set_T_stop(_vehicle->T());
        _open_results.erase(name);
    }
}

// make sure someone running a PixHawk hasn't just copied parameters
// from an APM:
void Analyzer_Parameters::evaluate_rc_channels()
{
    for (uint8_t channel=1; channel<=14; channel++) {
        float min;
        float max;
        float trim;
        const std::string name = string_format("RC%u", channel);
        const std::string name_min = string_format("RC%u_MIN", channel);
        const std::string name_max = string_format("RC%u_MAX", channel);
        const std::string name_trim = string_format("RC%u_TRIM", channel);
        if (! _vehicle->param(name_min, min) ||
            ! _vehicle->param(name_max, max) |
            ! _vehicle->param(name_trim, trim)) {
            continue;
        }
        bool bad = (max < min ||
                    trim < min ||
                    trim > max);
        if (bad) {
            if (!_open_results_rc_channels[name]) {
                open_result_rc_channel(name, min, max, trim);
            }
            update_result_rc_channel(name, min, max, trim);
        } else {
            if (_open_results_rc_channels[name]) {
                _open_results_rc_channels[name]->set_T_stop(_vehicle->T());
                _open_results_rc_channels[name] = nullptr;
            }
        }
    }
}

void Analyzer_Parameters::evaluate()
{
    evaluate_trivial();

    evaluate_log_bitmask();
    evaluate_rc_channels();
}

void Analyzer_Parameters::end_of_log(const uint32_t packet_count UNUSED)
{
    auto next = _open_results.begin();
    while (next != _open_results.end()) {
        auto current = next;
        next++;
        close_result_trivial(current->first);
    }
    if (_log_bitmask_bad_apm != nullptr) {
        _log_bitmask_bad_apm->set_T_stop(_vehicle->T());
    }
    auto xnext = _open_results_rc_channels.begin();
    while (xnext != _open_results_rc_channels.end()) {
        auto current = xnext;
        xnext++;
        close_result_rc_channel(current->first);
    }
}

bool Analyzer_Parameters::configure(INIReader *config)
{
    // FIXME: this is a bit hacky!
    _enable_ANGLE_MAX_check = is_equal(config->GetReal("loganalyzer", "parameters::check::ANGLE_MAX::enable", 1.0f), 1.0f);
    return true;
}
