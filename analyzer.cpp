#include "analyzer.h"

void Analyzer_Result::to_json_add_array(Json::Value &root,
                                        std::string name,
                                        std::vector<std::string> array) const
{
    if (array.size() == 0) {
        return;
    }
    Json::Value json_array(Json::arrayValue);

    for (std::vector<std::string>::const_iterator it = array.begin();
         it != array.end(); it++) {
        json_array.append((*it));
    }

    root[name] = json_array;
}

void Analyzer_Result::to_json(Json::Value &root) const
{
    root["status"] = status_as_string();
    root["evilness"] = evilness();
    root["severity-score"] = evilness();

    const std::string *my_reason = reason();
    if (my_reason != NULL) {
        root["reason"] = *my_reason;
    }

    std::vector<std::string> some_series;
    for (std::vector<const Data_Source*>::const_iterator it = _sources.begin();
         it != _sources.end();
         it++) {
        std::vector<std::string> source_series = (*it)->series();
        some_series.insert(some_series.end(), source_series.begin(), source_series.end());
    }
    to_json_add_array(root, "series", some_series);
    to_json_add_array(root, "evidence", _evidence);
}

void Analyzer_Result_Period::to_json(Json::Value &root) const
{
    Analyzer_Result::to_json(root);
    root["timestamp_start"] = (Json::UInt64)_T_start;
    root["timestamp_stop"] = (Json::UInt64)_T_stop;
    root["duration"] = (_T_stop - _T_start) / 1000000.0f;
    root["duration-units"] = "seconds";
}

void Analyzer_Result_Event::to_json(Json::Value &root) const
{
    Analyzer_Result::to_json(root);
    root["timestamp"] = (Json::UInt64)_T;
}


uint32_t Analyzer::evilness() const {
    uint32_t ret = 0; // remove _evilness here?
    std::vector<Analyzer_Result*> my_results = results();
    for (std::vector<Analyzer_Result*>::const_iterator it = my_results.begin();
         it != my_results.end();
         it++) {
        ret += (*it)->evilness();
    }
    return ret;
}

void Analyzer::results_json_results(Json::Value &root)
{
    std::vector<Analyzer_Result*> my_results = results();
    for (std::vector<Analyzer_Result*>::const_iterator it = my_results.begin();
         it != my_results.end();
         it++) {
        Json::Value result(Json::objectValue);
        (*it)->to_json(result);
        if (result["series"].type() == Json::nullValue) {
            ::fprintf(stderr, "No series in (%s)\n", name().c_str());
            // that appears to autovivify :-/
            result["series"] = Json::Value(Json::arrayValue);
        }
        if (result["reason"].type() == Json::nullValue) {
            ::fprintf(stderr, "No reason in (%s)\n", name().c_str());
        }
        if (result["severity-score"].type() == Json::nullValue) {
            ::fprintf(stderr, "No severity-score in (%s)\n", name().c_str());
        }
        root.append(result);
    }
}

analyzer_status Analyzer::status()
{
    analyzer_status ret = analyzer_status_ok;

    std::vector<Analyzer_Result*> my_results = results();
    for (std::vector<Analyzer_Result*>::const_iterator it = my_results.begin();
         it != my_results.end();
         it++) {
        switch((*it)->status()) {
        case analyzer_status_fail:
            ret = analyzer_status_fail;
            break;
        case analyzer_status_warn:
            if (ret == analyzer_status_ok) {
                ret = analyzer_status_warn;
            }
            break;
        case analyzer_status_ok:
            break;
        }
    }
    return ret;
}


const char *_status_as_string(analyzer_status status) {
    switch(status) {
    case analyzer_status_fail:
        return "FAIL";
    case analyzer_status_warn:
        return "WARN";
    case analyzer_status_ok:
        return "PASS";
    }
    return "STRANGE";
}
