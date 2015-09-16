#include "analyzer_vehicle_definition.h"

#include "util.h"

void Analyzer_Vehicle_Definition::evaluate()
{
    // ::fprintf(stderr, "vehicletype=%d\n", _vehicle->vehicletype());
    if (_vehicle->vehicletype() != AnalyzerVehicle::Base::vehicletype_t::invalid) {
        vehicle_invalid = false;
    }
}

void Analyzer_Vehicle_Definition::end_of_log(const uint32_t packet_count)
{
    Analyzer_Vehicle_Definition_Result *result = new Analyzer_Vehicle_Definition_Result();
    if (vehicle_invalid) {
        result->set_reason("No information provided defined what type of vehicle was being analysed");
        result->set_status(analyzer_status_fail);
        result->add_series(_data_sources.get("VEHICLE_DEFINITION"));
        result->add_evilness(50);
    } else {
        result->set_reason("Vehicle was appropriately defined");
        result->set_status(analyzer_status_ok);
    }
    
    add_result(result);
}
