#ifndef ANALYZING_DATAFLASH_MESSAGE_HANDLER_H
#define ANALYZING_DATAFLASH_MESSAGE_HANDLER_H

/*
 * dataflash_message_handler
 *
 * A base class for objects which process dataflash messages and
 * possibly send responses
 *
 */

#include <stdint.h>

#include "INIReader.h"

#include "dataflash_message_handler.h"
#include "LA_MsgHandler.h"

#include "analyze.h"

class Analyzing_DataFlash_Message_Handler : public DataFlash_Message_Handler {
public:
    Analyzing_DataFlash_Message_Handler(Analyze *analyze,
                                        AnalyzerVehicle::Base *&vehicle) :
        DataFlash_Message_Handler(),
        _analyze(analyze),
        _vehicle(vehicle) { }

    void handle_format_message_received(const char *name, const struct log_Format &format, const char *msg);
    void handle_message_received(const struct log_Format &format, const uint8_t *msg);

    void end_of_log(uint32_t packet_count) override;

protected:
private:
    Analyze *_analyze;
    AnalyzerVehicle::Base *&_vehicle;

    #define MAX_FORMATS 256
    struct LA_MsgHandler *handlers[MAX_FORMATS] = { };

    // FIXME: find a general way to do this:
    bool have_pos = false;
    LA_MsgHandler_AHR2 *ahr2_handler = NULL;
};

#endif
