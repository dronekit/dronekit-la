#include "analyzer_compass_vector_length.h"

#include <stdio.h>
#include "util.h"
#include "analyzer_util.h"
#include <algorithm>

#include "analyzervehicle_copter.h"

bool Analyzer_Compass_Vector_Length::configure(INIReader *config) {
    if (!Analyzer::configure(config)) {
        return false;
    }
    
    length_short_warn = config->GetReal("loganalyzer", "compass_vector_length::length_short_warn", 120.0f);
    length_short_fail = config->GetReal("loganalyzer", "compass_vector_length::length_short_fail", 100.0f);

    length_long_warn = config->GetReal("loganalyzer", "compass_vector_length::length_long_warn", 550.0f);
    length_long_fail = config->GetReal("loganalyzer", "compass_vector_length::length_long_fail", 600.0f);

    delta_warn_percent = config->GetReal("loganalyzer", "compass_vector_length::delta_warn", 25.0f);
    delta_fail_percent = config->GetReal("loganalyzer", "compass_vector_length::delta_fail", 35.0f);

    duration_min = config->GetReal("loganalyzer", "compass_vector_length::duration_min", 250000);

    return true;
}

void Analyzer_Compass_Vector_Length::open_result_short(const std::string name,
                                                       const double length)
{
    _result_short[name] = new Analyzer_Compass_Vector_Length_Result_Short(name);

    Analyzer_Compass_Vector_Length_Result_Short *result = _result_short[name];

    result->set_T_start(_vehicle->T());
    result->set_reason("Compass Vector Length below threshold");
    result->add_source(_data_sources.get(string_format("MAGNETIC_FIELD_STRENGTH_%s", name.c_str())));

    result->set_status(analyzer_status_warn);
    result->set_severity_score(5);
    
    update_result_short(result, length);
}

void Analyzer_Compass_Vector_Length::update_result_short
(Analyzer_Compass_Vector_Length_Result_Short *result, const double length)
{
    if (result->_length_min < 0 || // -1 is used as a "not updated yet" length
        length < result->_length_min) {
        result->_length_min = length;
        if (length < length_short_fail) {
            result->set_status(analyzer_status_fail);
            result->set_severity_score(10);
        }
    }
}

void Analyzer_Compass_Vector_Length::close_result_short(Analyzer_Compass_Vector_Length_Result_Short *result)
{
    result->set_T_stop(_vehicle->T());

    if (result->duration() < duration_min) {
        _result_short.erase(result->name());
        delete result;
        return;
    }
  
    if (result->_length_min < length_short_fail) {
        result->set_status(analyzer_status_fail);
        result->add_evidence(string_format("threshold=%f", length_short_fail));
    } else if (result->_length_min < length_short_warn) {
        result->set_status(analyzer_status_warn);
        result->add_evidence(string_format("threshold=%f", length_short_warn));
    }

    result->add_evidence(string_format("threshold-duration=%f seconds", duration_min/1000000.0f));
    result->add_evidence(string_format("length-min=%f", result->length_min()));

    add_result(result);
    _result_short.erase(result->name());
}

void Analyzer_Compass_Vector_Length::open_result_long(const std::string name,
                                                       const double length)
{
    _result_long[name] = new Analyzer_Compass_Vector_Length_Result_Long(name);

    Analyzer_Compass_Vector_Length_Result_Long *result = _result_long[name];

    result->set_T_start(_vehicle->T());
    result->set_reason("Compass Vector Length above threshold");
    result->add_source(_data_sources.get(string_format("MAGNETIC_FIELD_STRENGTH_%s", name.c_str())));

    result->set_status(analyzer_status_warn);
    result->set_severity_score(5);
    
    result->_length_max = 0;
    update_result_long(result, length);
}

void Analyzer_Compass_Vector_Length::update_result_long
(Analyzer_Compass_Vector_Length_Result_Long *result, const double length)
{
    if (length > result->_length_max) {
        result->_length_max = length;
        if (length >= length_long_fail) {
            result->set_status(analyzer_status_fail);
            result->set_severity_score(10);
        }
    }
}

void Analyzer_Compass_Vector_Length::close_result_long(Analyzer_Compass_Vector_Length_Result_Long *result)
{
    result->set_T_stop(_vehicle->T());

    if (result->duration() < duration_min) {
        _result_long.erase(result->name());
        delete result;
        return;
    }
  
    if (result->_length_max >= length_long_fail) {
        result->set_status(analyzer_status_fail);
        result->add_evidence(string_format("threshold=%f", length_long_fail));
    } else if (result->_length_max >= length_long_warn) {
        result->set_status(analyzer_status_warn);
        result->add_evidence(string_format("threshold=%f", length_long_warn));
    }

    result->add_evidence(string_format("threshold-duration=%f seconds", duration_min/1000000.0f));
    result->add_evidence(string_format("length-max=%f", result->length_max()));

    add_result(result);
    _result_long.erase(result->name());
}


void Analyzer_Compass_Vector_Length::evaluate_compass_field_length(AnalyzerVehicle::Compass *compass)
{
    if (!compass->field_T()) {
        return;
    }

    double length = compass->field().len();

// ::fprintf(stderr,"%lu compass vector length (%s)=%f (warn=%f) (fail=%f)\n",_vehicle->T(), compass->name().c_str(), length, length_short_warn, length_short_fail);

    // update the longest/shortest vectors ever seen:
    if (length > _longest_vector_length_seen[compass->name()]) {
        _longest_vector_length_seen[compass->name()] = length;
    }
    if (_shortest_vector_length_seen.count(compass->name()) == 0 ||
        length < _shortest_vector_length_seen[compass->name()]) {
        _shortest_vector_length_seen[compass->name()] = length;
    }

    // check for very short vectors
    if (! _result_short.count(compass->name())) {
        if (length <= length_short_warn) {
            // start a new incident
            open_result_short(compass->name(), length);
        }
    } else {
        // incident is currently underway
        if (length > length_short_warn) {
            close_result_short(_result_short[compass->name()]);
        } else {
            update_result_short(_result_short[compass->name()], length);
        }
    }

    // check for very long vectors
    if (! _result_long.count(compass->name())) {
        if (length >= length_long_warn) {
            // start a new incident
            open_result_long(compass->name(), length);
        }
    } else {
        // incident is currently underway
        if (length < length_long_warn) {
            close_result_long(_result_long[compass->name()]);
        } else {
            update_result_long(_result_long[compass->name()], length);
        }
    }
}

void Analyzer_Compass_Vector_Length::evaluate_compass(AnalyzerVehicle::Compass *compass)
{
    evaluate_compass_field_length(compass);
}

void Analyzer_Compass_Vector_Length::evaluate()
{
    std::for_each(_vehicle->compasses().begin(),
                  _vehicle->compasses().end(),
                      [this](std::pair<const std::string, AnalyzerVehicle::Compass*>c){ evaluate_compass(c.second); });
}

void Analyzer_Compass_Vector_Length::check_vector_delta(const std::string name,
                                                        const double shortest,
                                                        const double longest)
{
    double delta_percent = (longest - shortest)*100 / shortest;
    // ::fprintf(stderr, "%s: delta_percent=%f\n", name.c_str(), delta_percent);
    if (delta_percent >= delta_warn_percent) {
        Analyzer_Result_Summary *result = new Analyzer_Result_Summary();
        if (delta_percent >= delta_fail_percent) {
            result->set_status(analyzer_status_fail);
            result->set_severity_score(10);
        } else {
            result->set_status(analyzer_status_warn);
            result->set_severity_score(5);
        }
        result->set_reason("Compass Vector Length delta exceeds threshold");
        result->add_evidence(string_format("shortest=%f", shortest));
        result->add_evidence(string_format("longest=%f", longest));
        result->add_evidence(string_format("delta-percent=%f", delta_percent));
        result->add_source(_data_sources.get(string_format("MAGNETIC_FIELD_STRENGTH_%s", name.c_str())));
        add_result(result);
    }
}

// void Analyzer_Compass_Vector_Length::for_each_tmping(
//     std::map<const std::basic_string<char>, Analyzer_Compass_Vector_Length_Result_Short*>::iterator start,
//     std::map<const std::basic_string<char>, Analyzer_Compass_Vector_Length_Result_Short*>::iterator end,
//     std::function<std::map<const std::basic_string<char>, Analyzer_Compass_Vector_Length_Result_Short*>::iterator> function)
// {
//     std::map<const std::basic_string<char>, Analyzer_Compass_Vector_Length_Result_Short*>::iterator next = start;
//     while (next != end) {
//         std::map<const std::basic_string<char>, Analyzer_Compass_Vector_Length_Result_Short*>::iterator current = next;
//         next++;
//         function(next);
//     }
// }

void Analyzer_Compass_Vector_Length::close_results_short()
{
    auto next = _result_short.begin();
    while (next != _result_short.end()) {
        auto current = next;
        next++;
        close_result_short((*current).second);
    }
}

void Analyzer_Compass_Vector_Length::close_results_long()
{
    auto next = _result_long.begin();
    while (next != _result_long.end()) {
        auto current = next;
        next++;
        close_result_long((*current).second);
    }
}

void Analyzer_Compass_Vector_Length::end_of_log(uint32_t packet_count UNUSED)
{
    // close off any existing result:
    close_results_short();
    close_results_long();

    // check to see how much the vector length varied:
    std::for_each(_longest_vector_length_seen.begin(),
                  _longest_vector_length_seen.end(),
                  [this](std::pair<const std::string, double>c){
                      check_vector_delta(c.first, _shortest_vector_length_seen[c.first], c.second);
                  });
    


    // if (_vehicle->roll_modtime() == 0) {
    //     Analyzer_Result_Summary *result = new Analyzer_Result_Summary();
    //     result->set_status(analyzer_status_warn);
    //     result->set_reason("Vehicle attitude never set");
    //     result->set_severity_score(5);
    //     result->add_source(_data_sources.get("ATTITUDE"));
    //     add_result(result);
    // }
}
