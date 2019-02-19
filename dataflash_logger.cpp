#include "dataflash_logger.h"

#include <signal.h>
#include <stdio.h> // for snprintf
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>

#include "la-log.h"
#include "util.h"
#include "mavlink_c_library/common/mavlink.h"

void DataFlash_Logger::sighup_received()
{
    logging_stop();
}

void DataFlash_Logger::idle_tenthHz()
{
    // the following isn't quite right given we seek around...
    if (logging_started) {
	la_log(LOG_DEBUG, "mh-dfl: Current log size: %lu", lseek(out_fd, 0, SEEK_CUR));
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
	    la_log(LOG_INFO, "mh-dfl: No data packets received for some time (now=%llu last=%llu).  Closing log.  Final log size: %lu", now_us,_last_data_packet_time, lseek(out_fd, 0, SEEK_CUR));
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
    mavlink_msg_remote_log_block_status_pack(this_system_id, this_component_id, &msg, sender_system_id, sender_component_id, seqno, status);
    _mavlink_writer->send_message(msg);
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

    this_system_id = config->GetInteger("dflogger", "source_system_id", this_system_id);
    this_component_id = config->GetInteger("dflogger", "source_component_id", this_component_id);

    target_system_id = config->GetInteger("dflogger", "target_system_id", target_system_id);
    target_component_id = config->GetInteger("dflogger", "target_component_id", target_component_id);

    return true;
}

bool DataFlash_Logger::make_new_log_filename(char *buffer, uint8_t bufferlen)
{
  uint8_t lastlog_buflen = 128;
  char lastlog_buf[128];
    // this was really beautiful, but I don't think SoloLink has an
    // RTC; it depends on GPS to update its clock (scribbled down
    // periodically?)

    // time_t t;

    // time(&t);
    // struct tm *timebits = gmtime(&t);

    // snprintf(buffer, bufferlen, "%s/%04d%02d%02d%02d%02d%02d.BIN",
    //          _log_directory_path,
    //          timebits->tm_year+1900,
    //          timebits->tm_mon+1,
    //          timebits->tm_mday,
    //          timebits->tm_hour,
    //          timebits->tm_min,
    //          timebits->tm_sec);


    memset(lastlog_buf, '\0', lastlog_buflen);
    snprintf(lastlog_buf, lastlog_buflen, "%s/LASTLOG.TXT", _log_directory_path);
    int fd;
    uint32_t num;
    if ((fd = open(lastlog_buf, O_RDONLY)) == -1) {
        if (errno != ENOENT) {
            // what?
            la_log(LOG_ERR, "Failed to open (%s) for reading: %s", lastlog_buf, strerror(errno));
            return false;
        }
        num = 1;
    } else {
	const uint8_t numbuf_len = 128;
	char numbuf[numbuf_len];
        memset(numbuf, '\0', numbuf_len);
        int bytes_read = read(fd, numbuf, numbuf_len);
        close(fd);
        if (bytes_read == -1) {
            return false;
        }
        num = strtoul(numbuf, NULL, 10);
        num++;
    }

    if ((fd = open(lastlog_buf, O_WRONLY|O_TRUNC|O_CREAT, 0777)) == -1) {
        // *shrug*  We will continue to overwrite, I guess...
        la_log(LOG_ERR, "mh-dfl: failed to open (%s): %s", lastlog_buf, strerror(errno));
    } else {
        const uint8_t outsize = 16;
        char out[outsize];
        memset(out, '\0', outsize);
        int towrite = snprintf(out, outsize, "%d\r\n", num);
        if (write(fd, out, towrite) != towrite) {
            la_log(LOG_ERR, "mh-dfl: failed to write to (%s): %s", lastlog_buf, strerror(errno));
        }
        close(fd);
    }

    snprintf(buffer, bufferlen, "%s/%d.BIN",
	     _log_directory_path,
	     num);

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

bool DataFlash_Logger::logging_start(mavlink_message_t &m,
                                     mavlink_remote_log_data_block_t &msg UNUSED)
{
    sender_system_id = m.sysid;
    sender_component_id = m.compid;
    sender_arm_status = ARM_STATUS_UNKNOWN;

    la_log_unsuppress();
    la_log(LOG_INFO, "mh-dfl: Starting log, target is (%d/%d), I am (%d/%d)",
	   sender_system_id, sender_component_id, this_system_id, this_component_id);
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

void DataFlash_Logger::handle_decoded_message(uint64_t T,
                                              mavlink_message_t &m,
                                              mavlink_heartbeat_t &msg)
{
    // Need to match against heartbeats from both the 'target' and 'sender' components, since AP does not send heartbeats for logical endpoints at present.
    if (m.sysid == sender_system_id &&
       (m.compid == sender_component_id || m.compid == target_component_id)) {

        arm_status_t new_sender_arm_status = (msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) ? ARM_STATUS_ARMED : ARM_STATUS_DISARMED;

        if (logging_started) {
            if (sender_arm_status == ARM_STATUS_ARMED &&
                new_sender_arm_status == ARM_STATUS_DISARMED) {
                // sender has moved from armed to disarmed state; stop
                // logging and let it restart naturally
                la_log(LOG_INFO, "mh-dfl: disarm detected, logging_stop");
                logging_stop();
            }
        } else {
            la_log(LOG_INFO, "Heartbeat received from %u/%u", m.sysid, m.compid);
        }

        sender_arm_status = new_sender_arm_status;
    }

    MAVLink_Message_Handler::handle_decoded_message(T, m, msg);
}

void DataFlash_Logger::handle_decoded_message(uint64_t T UNUSED,
                                              mavlink_message_t &m,
                                              mavlink_remote_log_data_block_t &msg)
{
    if (!logging_started) {
	if (msg.seqno == 0) {
	    if (!logging_start(m, msg)) {
		return;
	    }
	} else {
	    return;
	}
    }

    // we could move this down to the end; that wold mean short-writes
    // would end up closing this log...
    _last_data_packet_time = clock_gettime_us(CLOCK_MONOTONIC);
    
    const uint8_t length = MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN;

    /* send the dataflash data out to the log file */
    lseek(out_fd,
	  msg.seqno*MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN,
	  SEEK_SET);
    if (write(out_fd, msg.data, length) < length) {
	la_log(LOG_ERR, "Short write: %s", strerror(errno));
	// we'll get the block again... maybe we'll have better luck next time..
	return;
    }

    // queue an ack for this packet
    queue_ack(msg.seqno);

    // queue nacks for gaps
    queue_gap_nacks(msg.seqno);

    if (msg.seqno > highest_seqno_seen) {
        if (msg.seqno - highest_seqno_seen > 100) {
            la_log(LOG_ERR, "large seqno gap: %ld", msg.seqno - highest_seqno_seen);
        }
	highest_seqno_seen = msg.seqno;
    }
}

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
	(this_system_id, this_component_id, &msg, system_id, component_id, magic_number, 1);

    _mavlink_writer->send_message(msg);
}
