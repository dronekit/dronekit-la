#include "common_tool.h"

#include "dataflash_logger.h"
#include "heart.h"

#include <signal.h>

#include "mavlink_reader.h"

#include "la-log.h"

#include <stdio.h> // for snprintf
#include <syslog.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>

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
	syslog(LOG_INFO, "Current log size: %ld", lseek(out_fd, 0, SEEK_CUR));
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
	    syslog(LOG_INFO, "No data packets received for some time (%ld/%ld).  Closing log.  Final log size: %ld", now_us,_last_data_packet_time, lseek(out_fd, 0, SEEK_CUR));
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

void DataFlash_Logger::handle_packet(uint8_t *pkt, uint16_t pktlen)
{
    /* manual unpacking! */
    // uint8_t seq = pkt[2];
    // uint8_t sys_id = pkt[3];
    // uint8_t comp_id = pkt[4];
    uint8_t msg_id = pkt[5];

    // ::printf("msg_id=%d\n", msg_id);
    if (msg_id == MAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK) {
	return handle_packet_remote_log_data_block(pkt, pktlen);
    }
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
	syslog(LOG_ERR, "Failed to open (%s): %s", filename, strerror(errno));
	return false;
    }
    syslog(LOG_INFO, "Opened log file (%s)", filename);

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
    syslog(LOG_INFO, "Starting log, target is (%d/%d)",
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

void DataFlash_Logger::handle_packet_remote_log_data_block(uint8_t *pkt, uint16_t pktlen)
{
    uint32_t seqno = *((uint32_t*)&pkt[6]);

    if (!logging_started) {
	if (seqno == 0) {
	    if (!logging_start(pkt, pktlen)) {
		return;
	    }
	} else {
	    return;
	}
    }

    // we could move this down to the end; that wold mean short-writes
    // would end up closing this log...
    _last_data_packet_time = clock_gettime_us(CLOCK_MONOTONIC);

    const uint8_t length = pkt[12];
    const uint8_t *data = &pkt[13];

    /* send the dataflash data out to the log file */
    lseek(out_fd,
	  seqno*MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN,
	  SEEK_SET);
    if (write(out_fd, data, length) < length) {
	syslog(LOG_ERR, "Short write: %s", strerror(errno));
	// we'll get the block again... maybe we'll have better luck next time..
	return;
    }

    // queue an ack for this packet
    queue_ack(seqno);

    // queue nacks for gaps
    queue_gap_nacks(seqno);

    if (seqno > highest_seqno_seen) {
	highest_seqno_seen = seqno;
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
	syslog(LOG_INFO, "sending start packet to (%d/%d)", system_id,component_id);
	magic_number = MAV_REMOTE_LOG_DATA_BLOCK_START;
    } else {
	syslog(LOG_INFO, "sending stop packet to (%d/%d)", system_id,component_id);
	magic_number = MAV_REMOTE_LOG_DATA_BLOCK_STOP;
    }
    mavlink_msg_remote_log_block_status_pack
	(system_id, component_id, &msg, system_id, component_id, magic_number, 1);

    send_message_to_telem_forwarder(msg);
}

class DataFlash_Logger_Program : Common_Tool {
public:
    DataFlash_Logger_Program() :
        Common_Tool(),
        use_telem_forwarder(false),
        _argc(0),
        _argv(NULL)
        { }
    void instantiate_message_handlers(INIReader *config,
                                      int fd_telem_forwarder,
                                      struct sockaddr_in *sa_tf);
    void sighup_handler(int signal);
    void run();
    void parse_arguments(int argc, char *argv[]);
    const char *program_name();

private:
    void usage();

    bool use_telem_forwarder;
    MAVLink_Reader *reader;

    long _argc;
    char **_argv;
};

const char *DataFlash_Logger_Program::program_name()
{
    if (_argv == NULL) {
        return "[Unknown]";
    }
    return _argv[0];
}

void DataFlash_Logger_Program::usage()
{
    ::printf("Usage:\n");
    ::printf("%s [OPTION] [FILE]\n", program_name());
    ::printf(" -c filepath      use config file filepath\n");
    ::printf(" -t               connect to telem forwarder to receive data\n");
    ::printf(" -s style         use output style (plain-text|json)\n");
    ::printf(" -h               display usage information\n");
    ::printf("\n");
    ::printf("Example: ./dataflash_logger -c /dev/null -s json 1.solo.tlog\n");
    exit(0);
}

void DataFlash_Logger_Program::instantiate_message_handlers(INIReader *config,
                                               int fd_telem_forwarder,
                                               struct sockaddr_in *sa_tf)
{
    DataFlash_Logger *dataflash_logger = new DataFlash_Logger(fd_telem_forwarder, sa_tf);
    if (dataflash_logger != NULL) {
        reader->add_message_handler(dataflash_logger, "DataFlash_Logger");
    } else {
        la_log(LOG_INFO, "Failed to create dataflash logger");
    }

    Heart *heart= new Heart(fd_telem_forwarder, sa_tf);
    if (heart != NULL) {
        reader->add_message_handler(heart, "Heart");
    } else {
        la_log(LOG_INFO, "Failed to create heart");
    }
}

DataFlash_Logger_Program logger;

void sighup_handler(int signal)
{
    logger.sighup_handler(signal);
}

void DataFlash_Logger_Program::sighup_handler(int signal)
{
    reader->sighup_handler(signal);
}
    
void DataFlash_Logger_Program::run()
{
    la_log(LOG_INFO, "dataflash_logger starting: built " __DATE__ " " __TIME__);
    signal(SIGHUP, ::sighup_handler);

    INIReader *config = get_config();

    reader = new MAVLink_Reader(config);
    if (reader == NULL) {
        la_log(LOG_ERR, "Failed to create reader from (%s)\n", config_filename);
        exit(1);
    }

    /* prepare sockaddr used to contact telem_forwarder */
    reader->pack_telem_forwarder_sockaddr(config);

    /* Prepare a port to receive and send data to/from telem_forwarder */
    /* does not return on failure */
    reader->create_and_bind();

    instantiate_message_handlers(config, reader->fd_telem_forwarder, &reader->sa_tf);
    return reader->telem_forwarder_loop();
}

void DataFlash_Logger_Program::parse_arguments(int argc, char *argv[])
{
    int opt;
    _argc = argc;
    _argv = argv;

    while ((opt = getopt(argc, argv, "hc:ts:")) != -1) {
        switch(opt) {
        case 'h':
            usage();
            break;
        case 'c':
            config_filename = optarg;
            break;
        case 't':
            use_telem_forwarder = true;
            break;
        }
    }
}



/*
* main - entry point
*/
int main(int argc, char* argv[])
{
    logger.parse_arguments(argc, argv);
    logger.run();
}
