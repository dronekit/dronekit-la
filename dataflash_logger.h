#ifndef DATAFLASH_LOGGER_H
#define DATAFLASH_LOGGER_H

/*
 * dataflash_logger
 *
 * Receive telemetry (mavlink) via UDP from Solo, and create dataflash log files
 *
 * Initiate a remote-dataflash stream
 */

#include "INIReader.h"

#include "mavlink_message_handler.h"

class DataFlash_Logger : public MAVLink_Message_Handler {

public:
    DataFlash_Logger(int fd, struct sockaddr_in *sa) :
	MAVLink_Message_Handler(fd, sa),
        target_system_id(target_system_id_default),
        target_component_id(target_component_id_default),
	sender_system_id(0),
	sender_component_id(0),
	logging_started(false),
	response_queue_head(0),
	response_queue_tail(0),
	seqno_gap_nack_threshold(20)
    {  }

    // this may change to handle_message(mavlink_message_t):
    // void handle_packet(uint8_t *pkt, uint16_t pktlen);
    bool configure(INIReader *config);

    void sighup_received();

    void send_stop_logging_packet();

    void idle_tenthHz();
    void idle_1Hz();
    void idle_10Hz();
    void idle_100Hz();

private:
    bool logging_start(uint8_t *pkt, uint16_t pktlen);
    void logging_stop();
    bool output_file_open();
    void output_file_close();

    const uint8_t target_system_id_default = 0;
    const uint8_t target_component_id_default = 0;
    
    uint8_t target_system_id;     // who to send our request-for-logs to
    uint8_t target_component_id;  // who to send our request-for-logs to
    uint8_t sender_system_id;     // who the logs areactually coming from
    uint8_t sender_component_id;  // who the logs areactually coming from
    void send_start_logging_packet();
    void send_start_or_stop_logging_packet(bool is_start);
    const char *_log_directory_path;
    int out_fd;
    bool logging_started;

    void handle_decoded_message(uint64_t T, mavlink_remote_log_data_block_t);

    bool make_new_log_filename(char *buffer, uint8_t bufferlen);

    void send_response(uint32_t seqno, bool status);
    void push_response_queue();

    #define RESPONSE_QUEUE_LENGTH 128
    struct packet_status {
	uint32_t seqno;
	bool status; // ack == true
    } responses[RESPONSE_QUEUE_LENGTH];
    uint8_t response_queue_head;
    uint8_t response_queue_tail;

    uint32_t highest_seqno_seen;
    uint64_t _last_data_packet_time;

    /* if we lose > this many packets we do not nack anything in that gap: */
    uint8_t seqno_gap_nack_threshold;

    void queue_response(uint32_t seqno, bool status);
    void queue_nack(uint32_t seqno);
    void queue_ack(uint32_t seqno);
    void queue_gap_nacks(uint32_t seqno);
};

#endif
