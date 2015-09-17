#include "LA_MsgHandler.h"

void LA_MsgHandler::process_set_T(const uint8_t *msg) {
    uint64_t time_us;
    uint64_t time_ms;
    if (field_value(msg, "TimeUS", time_us)) {
    } else if (field_value(msg, "TimeMS", time_ms)) {
        // this is going to screw up on GPS messages...
        time_us = time_ms * 1000;
    } else {
        // no timestamp...
        return;
    }
    char name[1024];
    string_for_labels(name, 1024);
    // ::fprintf(stderr, "type=%s T=%lu\n", name, time_us);
    _vehicle->set_T(time_us);
}


