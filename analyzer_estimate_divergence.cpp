#include "analyzer_estimate_divergence.h"

#include "analyzer_util.h"

void Analyzer_Estimate_Divergence_Result::to_json(Json::Value &root) const
{
    Analyzer_Result_Period::to_json(root);
    root["name"] = _name;
}

void Analyzer_Estimate_Divergence::end_of_log(const uint32_t packet_count)
{
    for (std::map<const std::string, Analyzer_Estimate_Divergence_Result*>::iterator it = _result.begin();
         it != _result.end();
         ++it) {
        Analyzer_Estimate_Divergence_Result *result = (*it).second;
        if (result != NULL) {
            close_result((*it).first);
        }        
    }
}

void Analyzer_Estimate_Divergence::update_result_set_status(Analyzer_Estimate_Divergence_Result *result)
{
    if (result->max_delta() >= delta_fail()) {
        result->set_status(analyzer_status_fail);
    } else if (result->max_delta() >= delta_warn()) {
        result->set_status(analyzer_status_warn);
    }
 }

void Analyzer_Estimate_Divergence::update_result(std::string name, double delta)
{
    Analyzer_Estimate_Divergence_Result *result = _result[name];
    if (delta > result->max_delta()) {
        result->set_max_delta(delta);

        update_result_set_status(result);

        if (result->status() == analyzer_status_fail) {
            result->set_delta_threshold(delta_fail());
            result->set_evilness(20);
        } else if (result->status() == analyzer_status_warn) {
            result->set_delta_threshold(delta_warn());
            result->set_evilness(10);
        }
    }
}

void Analyzer_Estimate_Divergence::close_result_add_evidence(Analyzer_Estimate_Divergence_Result *result)
{
    result->add_evidence(string_format("max-delta=%f metres", result->max_delta()));
    result->add_evidence(string_format("delta-threshold=%f metres", result->delta_threshold()));
    result->add_evidence(string_format("delta-time-threshold=%f seconds", delta_time_threshold() / 1000000.0f));
}

void Analyzer_Estimate_Divergence::close_result(std::string name)
{
    _result[name]->set_T_stop(_vehicle->T());
    if (_result[name]->duration() < delta_time_threshold()) {
        delete _result[name];
        _result[name] = NULL;
        return;
    }

    close_result_add_evidence(_result[name]);

    add_result(_result[name]);
    _result[name] = NULL;
}
