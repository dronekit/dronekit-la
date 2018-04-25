#include <signal.h>
#include <stdio.h> // for snprintf
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>

#include "la-log.h"
#include "util.h"
#include "mavlink_c_library/ardupilotmega/mavlink.h"

#include "log_list.h"

bool Log_List::configure(INIReader *config)
{
    if (!MAVLink_Message_Handler::configure(config)) {
        return false;
    }

    return true;
}

void Log_List::handle_message(uint64_t timestamp, mavlink_message_t &msg)
{
    most_recent_sender_system_id = msg.sysid;
    most_recent_sender_component_id = msg.compid;
    MAVLink_Message_Handler::handle_message(timestamp, msg);
}

void Log_List::handle_decoded_message(uint64_t T UNUSED, mavlink_log_entry_t &msg)
{
    // la_log(LOG_ERR, "entry: %u/%u", msg.id, msg.last_log_num);
    num_logs = msg.num_logs;
    if (logs.count(msg.id) == 0) {
        logs[msg.id] = new LogEntry();
    }
    logs[msg.id]->id = msg.id;
    logs[msg.id]->size = msg.size;
    logs[msg.id]->time_utc = msg.time_utc;
    seen_log_entry = true;
}

uint64_t Log_List::time_in_state() const
{
    return (clock_gettime_us(CLOCK_MONOTONIC) - state_start_time);
}

void Log_List::send_log_request_list()
{
    la_log(LOG_INFO, "mh-ll: sending log_request_list to %u/%u", target_system_id, target_component_id);

    mavlink_message_t msg;
    mavlink_msg_log_request_list_pack(system_id,
                                      component_id,
                                      &msg,
                                      target_system_id,
                                      target_component_id,
                                      0,
                                      0xFFFF);

    _mavlink_writer->send_message(msg);
}

void Log_List::print_entries()
{
    for (auto it = logs.begin(); it != logs.end(); ++it) {
        auto pair = *it;
        LogEntry *entry = pair.second;
        ::fprintf(stdout, "%5u %u %u\n",
                  entry->id,
                  entry->time_utc,
                  entry->size);
    }
}


void Log_List::run_statemachine()
{
    switch(state) {
    case WAITING_FOR_SYSTEM:
        if (target_system_id == 0) {
            if (most_recent_sender_component_id == 0) {
                return;
            }
        }
        target_system_id = most_recent_sender_system_id;
        target_component_id = most_recent_sender_component_id;
        state = SEND_LOG_REQUEST_LIST;
        // fall-through
    case SEND_LOG_REQUEST_LIST:
        send_log_request_list();
        state = SENT_LOG_REQUEST_LIST;
        // fall through
    case SENT_LOG_REQUEST_LIST:
        if (seen_log_entry &&
            num_logs == logs.size()) {
            state = OUTPUT;
            return;
        }
        if (time_in_state() < 1000000) {
            return;
        }
        state = SEND_LOG_REQUEST_LIST;
        return;
    case OUTPUT:
        print_entries();
        state = DONE;
        // fall-through
    case DONE:
        exit(0);
    }
}

void Log_List::idle_10Hz()
{
    run_statemachine();
    if (state != last_state) {
        last_state = state;
        state_start_time = clock_gettime_us(CLOCK_MONOTONIC);
    }
}
