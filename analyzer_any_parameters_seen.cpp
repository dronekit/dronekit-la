#include "analyzer_any_parameters_seen.h"

#include "util.h"

void Analyzer_Any_Parameters_Seen::evaluate()
{
    if (_vehicle->param_count() > 0) {
        any_parameters_seen = true;
    }
}

void Analyzer_Any_Parameters_Seen::results_json_results(Json::Value &root)
{
    Json::Value result(Json::objectValue);
    
    result["reason"] = any_parameters_seen ? "Parameters seen" : "No parameters seen";
    if (any_parameters_seen) {
        result["status"] =  "OK";
        result["severity-score"] = 0;
        result["evilness"] = result["severity-score"];
    } else {
        Json::Value series(Json::arrayValue);
        series.append("PARAM");
        result["series"] = series;
        result["severity-score"] = 20;
        result["evilness"] = result["severity-score"];
        result["status"] =  "FAIL";
    }

    root.append(result);
}
