#include "analyzer_vehicle_definition.h"

#include "util.h"

void Analyzer_Vehicle_Definition::evaluate()
{
    // ::fprintf(stderr, "vehicletype=%d\n", _vehicle->vehicletype());
    if (_vehicle->vehicletype() != AnalyzerVehicle::Base::vehicletype_t::invalid) {
        vehicle_invalid = false;
    }
}

void Analyzer_Vehicle_Definition::results_json_results(Json::Value &root)
{
    Json::Value result(Json::objectValue);
    
    if (vehicle_invalid) {
        result["reason"] = "No information provided defined what type of vehicle was being analysed";
        result["severity-score"] = 50;
        result["evilness"] = result["severity-score"];
        result["status"] =  "FAIL";
        Json::Value series(Json::arrayValue);
        series.append("MESSAGE");
        result["series"] = series;
    } else {
        result["reason"] = "Vehicle was appropriately defined";
        result["severity-score"] = 0;
        result["evilness"] = result["severity-score"];
        result["status"] =  "PASS";
    }
    root.append(result);
}
