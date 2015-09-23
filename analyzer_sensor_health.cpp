#include "analyzer_sensor_health.h"

#include "util.h"
#include "analyzer_util.h"

void Analyzer_Sensor_Health_Result::to_json(Json::Value &root) const
{
    Analyzer_Result_Period::to_json(root);
    root["sensor-name"] = _sensor_name;
}

bool Analyzer_Sensor_Health::configure(INIReader *config) {
    if (!Analyzer::configure(config)) {
	return false;
    }
    return true;
}

void Analyzer_Sensor_Health::end_of_log(uint32_t packet_count)
{
    std::map<const std::string, bool> sensors_health = _vehicle->sensors_health();

    bool have = false;
    for (std::map<const std::string, bool>::const_iterator it = sensors_health.begin();
         it != sensors_health.end();
         ++it) {
        // ::fprintf(stderr, "would evaluate (%s)\n", (*it).first.c_str());
        std::string name = (*it).first;
        if (_results[name] != NULL) {
            close_result(name);
        }
        have = true;
    }

    if (!have) {
        Analyzer_Result_Summary *summary = new Analyzer_Result_Summary();
        summary->set_status(analyzer_status_warn);
        summary->set_reason("Sensor health never updated");
        summary->add_series(_data_sources.get("SENSOR_HEALTH"));
        add_result(summary);
    }

}

void Analyzer_Sensor_Health::close_result(const std::string name)
{
    Analyzer_Sensor_Health_Result *result = _results[name];

    result->set_T_stop(_vehicle->T());
    // result->add_series(_data_sources.get(string_format("EKF_VARIANCES_%s", name.c_str())));
    // result->add_evidence(string_format("max-variance=%f", result->max()));
    add_result(result);
    _results[name] = NULL;
}

void Analyzer_Sensor_Health::open_result(const std::string name)
{
    _results[name] = new Analyzer_Sensor_Health_Result(name);
    _results[name]->set_T_start(_vehicle->T()); 
    _results[name]->set_status(analyzer_status_fail);
    // _result_flags->add_series(_data_sources.get("EKF_FLAGS"));
    // _result_flags->set_reason("The EKF status report indicates a problem with the EKF");

    // _result_flags->add_evidence(string_format("flags=%d", flags));
    // TODO: put the flags and the "bad" descrption into a structure
    _results[name]->add_evilness(10);
}


void Analyzer_Sensor_Health::evaluate()
{
    std::map<const std::string, bool> sensors_health = _vehicle->sensors_health();

    for (std::map<const std::string, bool>::const_iterator it = sensors_health.begin();
         it != sensors_health.end();
         ++it) {
        std::string name = (*it).first;
        // ::fprintf(stderr, "would evaluate (%s)\n", name.c_str());
        if ((*it).second) {
            if (_results[name] != NULL) {
                close_result(name);
            }
        } else {
            if (_results[name] == NULL) {
                open_result(name);
            }
        }
    }
}
