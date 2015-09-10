#include "dataflash_logger.h"

#include <signal.h>
#include <stdio.h> // for snprintf
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>

#include "la-log.h"
#include "util.h"
#include "mavlink/c_library/common/mavlink.h"

void DataFlash_Logger::sighup_received()
{
    logging_stop();
}

void DataFlash_Logger::idle_tenthHz()
{
    // the following isn't quite right given we seek around...
    if (logging_started) {
	la_log(LOG_INFO, "mh-dfl: Current log size: %ld", lseek(out_fd, 0, SEEK_CUR));
    }
}

void DataFlash_Logger::idle_1Hz()
{
   if (!logging_started) {
       if (sender_system_id != 0) {
	   // we've previously been logging, so telling the other end
	   // to stop logging may let us restart logging sooner
	   send_stop_logging_packet();
       }
       send_start_logging_packet();
   }
}
void DataFlash_Logger::idle_10Hz()
{
    if (logging_started) {
	// if no data packet in 10 seconds then close log
        uint64_t now_us = clock_gettime_us(CLOCK_MONOTONIC);
	if (now_us - _last_data_packet_time > 10000000) {
	    la_log(LOG_INFO, "mh-dfl: No data packets received for some time (%ld/%ld).  Closing log.  Final log size: %ld", now_us,_last_data_packet_time, lseek(out_fd, 0, SEEK_CUR));
	    logging_stop();
	}
    }
}

void DataFlash_Logger::idle_100Hz()
{
    push_response_queue();
}

void DataFlash_Logger::send_response(uint32_t seqno, bool status) {
    mavlink_message_t msg;
    mavlink_msg_remote_log_block_status_pack(system_id, component_id, &msg, sender_system_id, sender_component_id, seqno, status);
    send_message_to_telem_forwarder(msg);
}

void DataFlash_Logger::push_response_queue()
{
    const uint8_t max_packets_to_send = 5;
    uint8_t packets_sent = 0;
    while (response_queue_head != response_queue_tail &&
	   packets_sent < max_packets_to_send) {
	send_response(responses[response_queue_tail].seqno,
		      responses[response_queue_tail].status);
	response_queue_tail++;
	if (response_queue_tail >= RESPONSE_QUEUE_LENGTH) {
	    response_queue_tail = 0;
	}
    }
}

bool DataFlash_Logger::configure(INIReader *config)
{
    if (!MAVLink_Message_Handler::configure(config)) {
        return false;
    }

    std::string path = config->Get("dflogger", "log_dirpath", "/log/dataflash");
    _log_directory_path = strdup(path.c_str());
    if (_log_directory_path == NULL) {
	return false;
    }

    target_system_id = config->GetInteger("dflogger", "target_system_id", 0);
    target_component_id = config->GetInteger("dflogger", "target_component_id", 0);

    return true;
}

bool DataFlash_Logger::make_new_log_filename(char *buffer, uint8_t bufferlen)
{
    time_t t;

    time(&t);
    struct tm *timebits = gmtime(&t);

    // open file descriptor
    snprintf(buffer, bufferlen, "%s/%04d%02d%02d%02d%02d%02d.BIN",
	     _log_directory_path,
	     timebits->tm_year+1900,
	     timebits->tm_mon+1,
	     timebits->tm_mday,
	     timebits->tm_hour,
	     timebits->tm_min,
	     timebits->tm_sec);

    return true;
}

bool DataFlash_Logger::output_file_open()
{
    const uint8_t filename_length = 64;
    char filename[filename_length];

    if (!make_new_log_filename(filename, filename_length)) {
	return false;
    }

    out_fd = open(filename, O_WRONLY|O_CREAT|O_TRUNC, 0777);
    if (out_fd == -1) {
	printf("Failed to open (%s): %s\n", filename, strerror(errno));
	la_log(LOG_ERR, "Failed to open (%s): %s", filename, strerror(errno));
	return false;
    }
    la_log(LOG_INFO, "Opened log file (%s)", filename);

    return true;
}

void DataFlash_Logger::output_file_close()
{
    close(out_fd);
}


void DataFlash_Logger::queue_response(uint32_t seqno, bool status)
{
    responses[response_queue_head].seqno = seqno;
    responses[response_queue_head].status = status;
    response_queue_head++;
    if (response_queue_head >= RESPONSE_QUEUE_LENGTH) {
	response_queue_head = 0;
    }
}

void DataFlash_Logger::queue_ack(uint32_t seqno)
{
    queue_response(seqno, true);
}

void DataFlash_Logger::queue_nack(uint32_t seqno)
{
    queue_response(seqno, false);
}

void DataFlash_Logger::queue_gap_nacks(uint32_t seqno)
{
    if (seqno <= highest_seqno_seen) {
	// this packet filled in a gap (or was a dupe)
	return;
    }

    if (seqno - highest_seqno_seen > seqno_gap_nack_threshold) {
	// we've seen some serious disruption, and lots of stuff
	// is probably going to be lost.  Do not bother NACKing
	// packets here and let the server sort things out
	return;
    }

    for (uint32_t i=highest_seqno_seen+1; i<seqno; i++) {
	queue_nack(i);
    }
}

bool DataFlash_Logger::logging_start(uint8_t *pkt, uint16_t pktlen)
{
    sender_system_id = pkt[3];
    sender_component_id = pkt[4];
    la_log(LOG_INFO, "mh-dfl: Starting log, target is (%d/%d)",
	   sender_system_id, sender_component_id);
    if (!output_file_open()) {
	return false;
    }

    logging_started = true;
    return true;
}
void DataFlash_Logger::logging_stop()
{
    output_file_close();
    logging_started = false;
}

void handle_decoded_message(uint64_t T, mavlink_remote_log_data_block_t &msg)
{
    abort();
}

// void DataFlash_Logger::handle_packet_remote_log_data_block(uint8_t *pkt, uint16_t pktlen)
// {
//     uint32_t seqno = *((uint32_t*)&pkt[6]);

//     if (!logging_started) {
// 	if (seqno == 0) {
// 	    if (!logging_start(pkt, pktlen)) {
// 		return;
// 	    }
// 	} else {
// 	    return;
// 	}
//     }

//     // we could move this down to the end; that wold mean short-writes
//     // would end up closing this log...
//     _last_data_packet_time = clock_gettime_us(CLOCK_MONOTONIC);

//     const uint8_t length = pkt[12];
//     const uint8_t *data = &pkt[13];

//     /* send the dataflash data out to the log file */
//     lseek(out_fd,
// 	  seqno*MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN,
// 	  SEEK_SET);
//     if (write(out_fd, data, length) < length) {
// 	la_log(LOG_ERR, "Short write: %s", strerror(errno));
// 	// we'll get the block again... maybe we'll have better luck next time..
// 	return;
//     }

//     // queue an ack for this packet
//     queue_ack(seqno);

//     // queue nacks for gaps
//     queue_gap_nacks(seqno);

//     if (seqno > highest_seqno_seen) {
// 	highest_seqno_seen = seqno;
//     }
// }

void DataFlash_Logger::send_start_logging_packet()
{
    send_start_or_stop_logging_packet(true);
}

void DataFlash_Logger::send_stop_logging_packet()
{
    send_start_or_stop_logging_packet(false);
}

void DataFlash_Logger::send_start_or_stop_logging_packet(bool is_start)
{
    mavlink_message_t msg;

    uint8_t system_id = is_start ? target_system_id : sender_system_id;
    uint8_t component_id = is_start ? target_component_id : sender_component_id;
    uint32_t magic_number;
    if (is_start) {
	la_log(LOG_INFO, "mh-dfl: sending start packet to (%d/%d)", system_id,component_id);
	magic_number = MAV_REMOTE_LOG_DATA_BLOCK_START;
    } else {
	la_log(LOG_INFO, "mh-dfl: sending stop packet to (%d/%d)", system_id,component_id);
	magic_number = MAV_REMOTE_LOG_DATA_BLOCK_STOP;
    }
    mavlink_msg_remote_log_block_status_pack
	(system_id, component_id, &msg, system_id, component_id, magic_number, 1);

    send_message_to_telem_forwarder(msg);
}
