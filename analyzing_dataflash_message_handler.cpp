#include "analyzing_dataflash_message_handler.h"

void Analyzing_DataFlash_Message_Handler::handle_format_message_received(const char *name, const struct log_Format &format, const char *msg) {
    uint8_t new_msg_type = ((struct log_Format*)(msg))->type;
    if (streq(name, "AHR2")) {
        ahr2_handler= new LA_MsgHandler_AHR2(name, format, _analyze, _vehicle);
        handlers[new_msg_type] = ahr2_handler;
        if (have_pos) {
            ahr2_handler->set_canonical_for_position(false);
        }
        if (have_orgn) {
            ahr2_handler->set_canonical_for_origin(false);
        }
    } else if (streq(name, "ATT")) {
        handlers[new_msg_type] = new LA_MsgHandler_ATT(name, format, _analyze, _vehicle);
    } else if (streq(name, "BARO")) {
        handlers[new_msg_type] = new LA_MsgHandler_BARO(name, format, _analyze, _vehicle);
    } else if (streq(name, "EKF1")) {
        handlers[new_msg_type] = new LA_MsgHandler_EKF1(name, format, _analyze, _vehicle);
    } else if (streq(name, "EKF4")) {
        handlers[new_msg_type] = new LA_MsgHandler_EKF4(name, format, _analyze, _vehicle);
    } else if (streq(name, "ERR")) {
        handlers[new_msg_type] = new LA_MsgHandler_ERR(name, format, _analyze, _vehicle);
    } else if (streq(name, "EV")) {
        handlers[new_msg_type] = new LA_MsgHandler_EV(name, format, _analyze, _vehicle);
    } else if (streq(name, "GPA")) {
        handlers[new_msg_type] = new LA_MsgHandler_GPA(name, "GPS", format, _analyze, _vehicle);
    } else if (streq(name, "GPA2")) {
        handlers[new_msg_type] = new LA_MsgHandler_GPA(name, "GPS2", format, _analyze, _vehicle);
    } else if (streq(name, "GPS")) {
        handlers[new_msg_type] = new LA_MsgHandler_GPS(name, format, _analyze, _vehicle);
    } else if (streq(name, "GPS2")) {
        handlers[new_msg_type] = new LA_MsgHandler_GPS(name, format, _analyze, _vehicle);
    } else if (streq(name, "IMU")) {
        handlers[new_msg_type] = new LA_MsgHandler_IMU(name, format, _analyze, _vehicle);
    } else if (streq(name, "IMU2")) {
        handlers[new_msg_type] = new LA_MsgHandler_IMU(name, format, _analyze, _vehicle);
    } else if (streq(name, "IMU3")) {
        handlers[new_msg_type] = new LA_MsgHandler_IMU(name, format, _analyze, _vehicle);
    } else if (streq(name, "MAG")) {
        handlers[new_msg_type] = new LA_MsgHandler_MAG(name, format, _analyze, _vehicle);
    } else if (streq(name, "MAG2")) {
        handlers[new_msg_type] = new LA_MsgHandler_MAG(name, format, _analyze, _vehicle);
    } else if (streq(name, "MAG3")) {
        handlers[new_msg_type] = new LA_MsgHandler_MAG(name, format, _analyze, _vehicle);
    } else if (streq(name, "MODE")) {
        handlers[new_msg_type] = new LA_MsgHandler_MODE(name, format, _analyze, _vehicle);
    } else if (streq(name, "MSG")) {
        handlers[new_msg_type] = new LA_MsgHandler_MSG(name, format, _analyze, _vehicle);
    } else if (streq(name, "NKF1")) {
        handlers[new_msg_type] = new LA_MsgHandler_NKF1(name, format, _analyze, _vehicle);
    } else if (streq(name, "ORGN")) {
        have_orgn = true;
        handlers[new_msg_type] = new LA_MsgHandler_ORGN(name, format, _analyze, _vehicle);
        if (ahr2_handler != NULL) {
            ahr2_handler->set_canonical_for_origin(false);
        }
    } else if (streq(name, "PARM")) {
        handlers[new_msg_type] = new LA_MsgHandler_PARM(name, format, _analyze, _vehicle);
    } else if (streq(name, "PM")) {
        handlers[new_msg_type] = new LA_MsgHandler_PM(name, format, _analyze, _vehicle);
    } else if (streq(name, "POS")) {
        handlers[new_msg_type] = new LA_MsgHandler_POS(name, format, _analyze, _vehicle);
        have_pos = true;
        if (ahr2_handler != NULL) {
            ahr2_handler->set_canonical_for_position(false);
        }
    } else if (streq(name, "POWR")) {
        handlers[new_msg_type] = new LA_MsgHandler_POWR(name, format, _analyze, _vehicle);
    } else if (streq(name, "RATE")) {
        handlers[new_msg_type] = new LA_MsgHandler_RATE(name, format, _analyze, _vehicle);
    } else if (streq(name, "RCOU")) {
        handlers[new_msg_type] = new LA_MsgHandler_RCOU(name, format, _analyze, _vehicle);
    } else if (streq(name, "STAT")) {
        handlers[new_msg_type] = new LA_MsgHandler_STAT(name, format, _analyze, _vehicle);
    } else if (streq(name, "UBX3")) {
        handlers[new_msg_type] = new LA_MsgHandler_UBX3(name, format, _analyze, _vehicle);
    } else if (streq(name, "VIBE")) {
        handlers[new_msg_type] = new LA_MsgHandler_VIBE(name, format, _analyze, _vehicle);
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

void Analyzing_DataFlash_Message_Handler::end_of_log(uint32_t packet_count, uint64_t bytes_dropped UNUSED)
{
    _analyze->end_of_log(packet_count);
}
