#ifndef LOG_LIST_H
#define LOG_LIST_H

/*
 * log_list
 *
 * download log data over a mavlink connection
 *
 */

#include "INIReader.h"

#include "mavlink_message_handler.h"
#include "mavlink_writer.h"

class Log_List : public MAVLink_Message_Handler {

public:
    Log_List(MAVLink_Writer *mavlink_writer) :
	MAVLink_Message_Handler(),
        _mavlink_writer(mavlink_writer),
        target_system_id(target_system_id_default),
        target_component_id(target_component_id_default)
    {  }

private:

    bool configure(INIReader *config);

    void idle_10Hz();

    MAVLink_Writer *_mavlink_writer = NULL;
    uint8_t this_system_id = 58;
    uint8_t this_component_id = 58;

    const uint8_t target_system_id_default = 0;
    const uint8_t target_component_id_default = 0;
    uint8_t most_recent_sender_system_id = 0;
    uint8_t most_recent_sender_component_id = 0;
    uint8_t target_system_id;     // who to send our request-for-logs to
    uint8_t target_component_id;  // who to send our request-for-logs to

    int out_fd;

    void handle_message(uint64_t timestamp, mavlink_message_t &msg) override;
    void handle_decoded_message(uint64_t T UNUSED, mavlink_log_entry_t &msg) override;

    enum state_t {
        WAITING_FOR_SYSTEM,
        SEND_LOG_REQUEST_LIST,
        SENT_LOG_REQUEST_LIST,
        DONE,
        OUTPUT,
    };

    state_t state = WAITING_FOR_SYSTEM;
    state_t last_state;
    uint64_t state_start_time;

    uint16_t num_logs;

    class LogEntry {
    public:
        uint16_t id;
        uint32_t time_utc;
        uint32_t size;
    };
    std::map<uint16_t, LogEntry*> logs;
    bool seen_log_entry = false;

    void send_log_request_list();
    void run_statemachine();
    uint64_t time_in_state() const;
    void print_entries();

};

#endif
