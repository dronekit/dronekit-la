#include "analyzer_vehicle_definition.h"

#include "util.h"

void Analyzer_Vehicle_Definition::evaluate()
{
    // ::fprintf(stderr, "vehicletype=%d\n", _vehicle->vehicletype());
    if (_vehicle->vehicletype() != AnalyzerVehicle::Base::vehicletype_t::invalid) {
        vehicle_invalid = false;
    }
}

void Analyzer_Vehicle_Definition::end_of_log(const uint32_t packet_count UNUSED)
{
    Analyzer_Vehicle_Definition_Result *result = new Analyzer_Vehicle_Definition_Result();
    result->add_source(_data_sources.get("VEHICLE_DEFINITION"));
    if (vehicle_invalid) {
        result->set_reason("No information provided defined what type of vehicle was being analysed");
        result->set_status(analyzer_status_fail);
        result->increase_severity_score(50);
    } else {
        result->set_reason("Vehicle was appropriately defined");
        result->add_evidence(_vehicle->typeString());
        result->set_status(analyzer_status_ok);
    }
    
    add_result(result);
}
