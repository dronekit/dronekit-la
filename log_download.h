#ifndef LOG_DOWNLOAD_H
#define LOG_DOWNLOAD_H

#include <vector>

/*
 * log_download
 *
 * download log data over a mavlink connection
 *
 */

#include "INIReader.h"

#include "mavlink_message_handler.h"
#include "mavlink_writer.h"

class Log_Download : public MAVLink_Message_Handler {

public:
    Log_Download(MAVLink_Writer *mavlink_writer, std::vector<uint16_t> _logs_to_download) :
	MAVLink_Message_Handler(),
        _mavlink_writer(mavlink_writer),
        target_system_id(target_system_id_default),
        target_component_id(target_component_id_default),
        logs_to_download(_logs_to_download)
    {  }

private:

    bool configure(INIReader *config);

    void idle_10Hz();

    MAVLink_Writer *_mavlink_writer = NULL;
    uint8_t this_system_id = 58;
    uint8_t this_component_id = 58;

    const uint8_t target_system_id_default = 0;
    const uint8_t target_component_id_default = 0;
    uint8_t most_recent_sender_system_id;
    uint8_t most_recent_sender_component_id;
    uint8_t target_system_id;     // who to send our request-for-logs to
    uint8_t target_component_id;  // who to send our request-for-logs to

    int output_fh = -1;

    void sort_gaps();

    void handle_message(uint64_t timestamp, mavlink_message_t &msg) override;
    void handle_decoded_message(uint64_t T UNUSED, mavlink_log_data_t &msg) override;
    void handle_decoded_message(uint64_t T UNUSED, mavlink_log_entry_t &msg) override;

    void send_log_request_list(uint16_t id);

    void write_data(mavlink_log_data_t &msg);

    std::vector<uint16_t> logs_to_download;
    uint16_t id_to_fetch;

    class LogEntry {
    public:
        uint16_t id;
        uint32_t time_utc;
        uint32_t size;
    };
    LogEntry *log_info = nullptr;

    class Gap {
    public:
        uint32_t start;
        uint32_t len;
    };
    std::vector<Gap*> gaps;

    uint64_t download_start_time;
    void send_data_request(Gap *gap);

    enum state_t {
        NEXT_LOG,
        WAITING_FOR_SYSTEM,
        SEND_GET_LOG_INFO,
        SENT_GET_LOG_INFO,
        FILL_GAPS,
        DONE,
    };
    state_t state = WAITING_FOR_SYSTEM;
    state_t last_state;
    uint64_t state_start_time;

    uint64_t last_data_receieved;
    void run_statemachine();

    uint64_t time_in_state() const;


};

#endif
