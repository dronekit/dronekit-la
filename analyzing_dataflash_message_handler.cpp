#include "analyzing_dataflash_message_handler.h"

void Analyzing_DataFlash_Message_Handler::handle_format_message_received(const char *name, const struct log_Format &format, const char *msg) {
    uint8_t new_msg_type = ((struct log_Format*)(msg))->type;
    if (streq(name, "ATT")) {
        handlers[new_msg_type] = new LA_MsgHandler_ATT(format, _analyze, _vehicle);
    } else if (streq(name, "EKF4")) {
        handlers[new_msg_type] = new LA_MsgHandler_EKF4(format, _analyze, _vehicle);
    } else if (streq(name, "ERR")) {
        handlers[new_msg_type] = new LA_MsgHandler_ERR(format, _analyze, _vehicle);
    } else if (streq(name, "EV")) {
        handlers[new_msg_type] = new LA_MsgHandler_EV(format, _analyze, _vehicle);
    } else if (streq(name, "MSG")) {
        handlers[new_msg_type] = new LA_MsgHandler_MSG(format, _analyze, _vehicle);
    } else if (streq(name, "PARM")) {
        handlers[new_msg_type] = new LA_MsgHandler_PARM(format, _analyze, _vehicle);
    } else if (streq(name, "RCOU")) {
        handlers[new_msg_type] = new LA_MsgHandler_RCOU(format, _analyze, _vehicle);
    } else {
        return;
    }
    if (handlers[new_msg_type] == NULL) {
        ::fprintf(stderr, "Failed to construct");
        abort();
    }
}


void Analyzing_DataFlash_Message_Handler::handle_message_received(const struct log_Format &format, const uint8_t *msg)
{

    LA_MsgHandler *handler = handlers[format.type];
    if (handler == NULL) {
        // ::fprintf(stderr, "No handler for (%u) (%s)\n", format.type, format_names[format.type]);
        return;
    }

    handler->process(msg);
}

void Analyzing_DataFlash_Message_Handler::end_of_log(uint32_t packet_count)
{
    _analyze->end_of_log(packet_count);
}
