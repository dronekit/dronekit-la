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
void Analyzer_Parameters::evaluate()
{
    evaluate_trivial();
}

void Analyzer_Parameters::end_of_log(const uint32_t packet_count UNUSED)
{
    auto next = _open_results.begin();
    while (next != _open_results.end()) {
        auto current = next;
        next++;
        close_result_trivial(current->first);
    }
}

bool Analyzer_Parameters::configure(INIReader *config)
{
    // FIXME: this is a bit hacky!
    _enable_ANGLE_MAX_check = is_equal(config->GetReal("loganalyzer", "parameters::check::ANGLE_MAX::enable", 1.0f), 1.0f);
    return true;
}
