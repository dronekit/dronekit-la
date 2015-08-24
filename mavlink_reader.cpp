#include "mavlink_reader.h"

#include "dataflash_logger.h"
#include "heart.h"

#include <errno.h>
#include <stdio.h> // for perror
#include <stdlib.h> // for abort
#include <sys/mman.h> // for MCL_ macros and mlockall
#include <syslog.h>

#include <unistd.h> // for usleep

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <signal.h>
#include <fcntl.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <dirent.h>

#define streq(a, b) (!strcmp(a, b))

static bool sighup_received = false;
void sighup_handler(int signal)
{
    sighup_received = true;
}

/* This is used to prevent swamping the log with error messages if
   something unexpected happens.
   Returns -1 if we cannot log an error now, or returns the number of
   messages skipped due to rate limiting if we can, i.e. a return of
   2 means log, and we have skipped 2 messages due to rate limiting. */
int MAVLink_Reader::can_log_error()
{
    unsigned ret_val;
    uint64_t now_us = clock_gettime_us(CLOCK_MONOTONIC);
    if ((now_us - err_time_us) < err_interval_us) {
        /* can't log */
        err_skipped++;
        return -1;
    }
    /* yes; say we can and set err_time_us assuming we do log something */
    err_time_us = now_us;
    ret_val = err_skipped;
    err_skipped = 0;
    return ret_val;
}


/*
* create_and_bind - create a socket and bind it to a local UDP port
*
* Used to create the socket on the upstream side that receives from and sends
* to telem_forwarder
*
* Returns fd on success, -1 on error.
*/
int MAVLink_Reader::create_and_bind()
{
    int fd;
    struct sockaddr_in sa;

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("socket");
        abort();
    }

    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_addr.s_addr = htonl(INADDR_ANY);
    sa.sin_port = 0; // we don't care what our port is

    if (bind(fd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
        perror("bind");
        abort();
    }

    return fd;
} /* create_and_bind */


void MAVLink_Reader::pack_telem_forwarder_sockaddr(INIReader *config)
{
    uint16_t tf_port = config->GetInteger("solo", "telem_forward_port", 14560);
    std::string ip = config->Get("solo", "soloIp", "10.1.1.10");

    memset(&sa_tf, 0, sizeof(sa_tf));
    sa_tf.sin_family = AF_INET;
    //    sa_tf.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    inet_aton(ip.c_str(), &sa_tf.sin_addr); // useful for debugging
    sa_tf.sin_port = htons(tf_port);
}

void MAVLink_Reader::configure_message_handler(INIReader *config,
                                               MAVLink_Message_Handler *handler,
                                               const char *handler_name)
{
    if (handler->configure(config)) {
        message_handler[next_message_handler++] = handler;
    } else {
        syslog(LOG_INFO, "Failed to configure (%s)", handler_name);
    }
}

void MAVLink_Reader::instantiate_message_handlers(INIReader *config)
{
    if (MAX_MESSAGE_HANDLERS - next_message_handler < 2) {
	syslog(LOG_INFO, "Insufficient message handler slots (MAX=%d) (next=%d)?!", MAX_MESSAGE_HANDLERS, next_message_handler);
	exit(1);
    }

    if (use_telem_forwarder) {
        DataFlash_Logger *dataflash_logger = new DataFlash_Logger(fd_telem_forwarder, sa_tf);
        if (dataflash_logger != NULL) {
            configure_message_handler(config, dataflash_logger, "DataFlash_Logger");
        } else {
            syslog(LOG_INFO, "Failed to create dataflash logger");
        }

        Heart *heart= new Heart(fd_telem_forwarder, sa_tf);
        if (heart != NULL) {
            configure_message_handler(config, heart, "Heart");
        } else {
            syslog(LOG_INFO, "Failed to create heart");
        }
    }

    Analyze *analyze = new Analyze(fd_telem_forwarder, sa_tf);
    if (analyze != NULL) {
        analyze->set_output_style(output_style);
        analyze->instantiate_analyzers(config);
        configure_message_handler(config, analyze, "Analyze");
    } else {
        syslog(LOG_INFO, "Failed to create analyze");
    }
}

void MAVLink_Reader::clear_message_handlers()
{
    next_message_handler = 0;
}

bool MAVLink_Reader::sane_telem_forwarder_packet(uint8_t *pkt, uint16_t pktlen)
{
    if (sa.sin_addr.s_addr != sa_tf.sin_addr.s_addr) {
	unsigned skipped;
	if ((skipped = can_log_error()) >= 0)
	    syslog(LOG_ERR, "[%u] received packet not from solo (0x%08x)",
		   skipped, sa.sin_addr.s_addr);
	return false;
    }
    if (pktlen < 8) { 
	unsigned skipped;
	if ((skipped = can_log_error()) >= 0)
	    syslog(LOG_ERR, "[%u] received runt packet (%d bytes)",
		   skipped, pktlen);
	return false;
    }
    if (pkt[0] != 254) {
	unsigned skipped;
	if ((skipped = can_log_error()) >= 0)
	    syslog(LOG_ERR, "[%u] received bad magic (0x%02x)",
		   skipped, pkt[0]);
	return false;
    }
    if (pkt[1] != (pktlen - 8)) {
	unsigned skipped;
	if ((skipped = can_log_error()) >= 0)
	    syslog(LOG_ERR, "[%u] inconsistent length (%u, %u)",
		   skipped, pkt[1], pktlen);
	return false;
    }
    return true;
}

void MAVLink_Reader::handle_message_received(uint64_t timestamp, mavlink_message_t msg)
{
    switch(msg.msgid) {
    case MAVLINK_MSG_ID_AHRS2: {
        mavlink_ahrs2_t decoded;
        mavlink_msg_ahrs2_decode(&msg, &decoded);
        handle_decoded_message_received(timestamp, decoded); // template in .h
        break;
    }
    case MAVLINK_MSG_ID_ATTITUDE: {
        mavlink_attitude_t decoded;
        mavlink_msg_attitude_decode(&msg, &decoded);
        handle_decoded_message_received(timestamp, decoded); // template in .h
        break;
    }
    case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_heartbeat_t decoded;
        mavlink_msg_heartbeat_decode(&msg, &decoded);
        handle_decoded_message_received(timestamp, decoded); // template in .h
        break;
    }
    case MAVLINK_MSG_ID_PARAM_VALUE: {
        mavlink_param_value_t decoded;
        mavlink_msg_param_value_decode(&msg, &decoded);
        handle_decoded_message_received(timestamp, decoded); // template in .h
        break;
    }
    case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: {
        mavlink_servo_output_raw_t decoded;
        mavlink_msg_servo_output_raw_decode(&msg, &decoded);
        handle_decoded_message_received(timestamp, decoded); // template in .h
        break;
    }
    case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
        mavlink_ekf_status_report_t decoded;
        mavlink_msg_ekf_status_report_decode(&msg, &decoded);
        handle_decoded_message_received(timestamp, decoded); // template in .h
        break;
    }
    case MAVLINK_MSG_ID_SYS_STATUS: {
        mavlink_sys_status_t decoded;
        mavlink_msg_sys_status_decode(&msg, &decoded);
        handle_decoded_message_received(timestamp, decoded); // template in .h
        break;
    }
    case MAVLINK_MSG_ID_VFR_HUD: {
        mavlink_vfr_hud_t decoded;
        mavlink_msg_vfr_hud_decode(&msg, &decoded);
        handle_decoded_message_received(timestamp, decoded); // template in .h
        break;
    }
    }
}

void MAVLink_Reader::handle_packet_received(uint8_t *pkt, uint16_t size)
{
    // for(int i=0; i<next_message_handler; i++) {
    //     message_handler[i]->handle_packet(pkt, res);
    // }
}

void MAVLink_Reader::handle_telem_forwarder_recv()
{
    // ::printf("Receiving packet\n");
    /* packet from telem_forwarder */
    uint8_t pkt[TELEM_PKT_MAX];
    socklen_t sa_len = sizeof(sa);
    uint16_t res = recvfrom(fd_telem_forwarder, pkt, sizeof(pkt), 0, (struct sockaddr*)&sa, &sa_len);

    /* We get one mavlink packet per udp datagram. Sanity checks here
       are: must be from solo's IP and have a valid mavlink header. */
    if (!sane_telem_forwarder_packet(pkt, res)) {
	return;
    }

    /* packet is from solo and passes sanity checks */
    handle_packet_received(pkt, res);
}

void MAVLink_Reader::do_idle_callbacks() {
    uint64_t now_us = clock_gettime_us(CLOCK_MONOTONIC);
    if (next_100hz_time <= now_us) {
	for(int i=0; i<next_message_handler; i++) {
	    message_handler[i]->idle_100Hz();
	}
	next_100hz_time += 10000;
    }
    if (next_10hz_time <= now_us) {
	for(int i=0; i<next_message_handler; i++) {
	    message_handler[i]->idle_10Hz();
	}
	next_10hz_time += 100000;
    }
    if (next_1hz_time <= now_us) {
	for(int i=0; i<next_message_handler; i++) {
	    message_handler[i]->idle_1Hz();
	}
	next_1hz_time += 1000000;
    }
    if (next_tenthhz_time <= now_us) {
	for(int i=0; i<next_message_handler; i++) {
	    message_handler[i]->idle_tenthHz();
	}
	next_tenthhz_time += 10000000;
    }
}

void MAVLink_Reader::usage()
{
    ::printf("Usage:\n");
    ::printf("%s [OPTION] [FILE]\n", "x");
    ::printf(" -c filepath      use config file filepath\n");
    ::printf(" -t               connect to telem forwarder to receive data\n");
    ::printf(" -s style         use output style (plain-text|json)\n");
    ::printf(" -h               display usage information\n");
    exit(0);
}

void MAVLink_Reader::parse_arguments(int argc, char *argv[])
{
    int opt;

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
        case 's':
            output_style_string = optarg;
            break;
        }
    }
    if (optind < argc) {
        _pathname = argv[optind++];
    }
}

void MAVLink_Reader::run()
{
    openlog("dl", LOG_NDELAY, LOG_LOCAL1);

    syslog(LOG_INFO, "dataflash_logger starting: built " __DATE__ " " __TIME__);
    signal(SIGHUP, sighup_handler);

    output_style = Analyze::OUTPUT_JSON;
    if (output_style_string != NULL) {
        if (streq(output_style_string, "json")) {
            output_style = Analyze::OUTPUT_JSON;
        } else if(streq(output_style_string, "plain-text")) {
            output_style = Analyze::OUTPUT_PLAINTEXT;
        } else if(streq(output_style_string, "html")) {
            output_style = Analyze::OUTPUT_HTML;
        } else {
            usage();
            exit(1);
        }
    }

    config = new INIReader(config_filename);
    if (config->ParseError() < 0) {
        syslog(LOG_CRIT, "can't parse (%s)", config_filename);
        usage(); // FIXME!
        exit(1);
    }

    if (use_telem_forwarder) {
        /* prepare sockaddr used to contact telem_forwarder */
        pack_telem_forwarder_sockaddr(config);

        /* Prepare a port to receive and send data to/from telem_forwarder */
        /* does not return on failure */
        fd_telem_forwarder = create_and_bind();
    }


    if (use_telem_forwarder) {
        instantiate_message_handlers(config);
        return telem_forwarder_loop();
    }
    if (_pathname != NULL) {
        parse_path(_pathname);
        return;
    }
    usage();
}

void MAVLink_Reader::telem_forwarder_loop()
{
    while (1) {
	if (sighup_received) {
	    for(int i=0; i<next_message_handler; i++) {
		message_handler[i]->sighup_received();
	    }
	    sighup_received = false;
	}
        /* Wait for a packet, or time out if no packets arrive so we always
           periodically log status and check for new destinations. Downlink
           packets are on the order of 100/sec, so the timeout is such that
           we don't expect timeouts unless solo stops sending packets. We
           almost always get a packet with a 200 msec timeout, but not with
           a 100 msec timeout. (Timeouts don't really matter though.) */

	struct timeval timeout;

        fd_set fds;
        fd_set fds_err;
        FD_ZERO(&fds);
        uint8_t nfds = 0;
        FD_SET(fd_telem_forwarder, &fds);
        if (fd_telem_forwarder >= nfds)
            nfds = fd_telem_forwarder + 1;
	fds_err = fds;

        timeout.tv_sec = 0;
        timeout.tv_usec = 200000;
        int res = select(nfds, &fds, NULL, &fds_err, &timeout);

        if (res < 0) {
            unsigned skipped;
            if ((skipped = can_log_error()) >= 0)
                syslog(LOG_ERR, "[%u] select: %s", skipped, strerror(errno));
            /* this sleep is to avoid soaking the CPU if select starts
               returning immediately for some reason */
	    /* previous code was not checking errfds; we are now, so
	       perhaps this usleep can go away -pb20150730 */
            usleep(10000);
            continue;
        }

        if (res == 0) {
	  // select timeout
        }

        /* check for packets from telem_forwarder */
        if (FD_ISSET(fd_telem_forwarder, &fds_err)) {
            unsigned skipped;
            if ((skipped = can_log_error()) >= 0)
                syslog(LOG_ERR, "[%u] select(fd_telem_forwarder): %s", skipped, strerror(errno));
	}

        if (FD_ISSET(fd_telem_forwarder, &fds)) {
   	    handle_telem_forwarder_recv();
        }

	do_idle_callbacks();
    } /* while (1) */
}

const char *examining_filename; // REMOVE ME!

void MAVLink_Reader::parse_path(const char *path)
{
    if (!strcmp(path, "-")) {
        instantiate_message_handlers(config);
        parse_fd(fileno(stdin));
        clear_message_handlers();
        return;
    }
    
    struct stat buf;
    if (stat(path, &buf) == -1) {
        fprintf(stderr, "Failed to stat (%s): %s\n", path, strerror(errno));
        exit(1);
    }

    switch (buf.st_mode & S_IFMT) {
    case S_IFREG:
        instantiate_message_handlers(config);
        parse_filepath(path);
        clear_message_handlers();
        return;
    case S_IFDIR:
        return parse_directory_full_of_files(path);
    default:
        fprintf(stderr, "Not a file or directory\n");
        exit(1);
    }
}

void MAVLink_Reader::parse_directory_full_of_files(const char *dirpath)
{
    DIR *dh = opendir(dirpath);
    if (dh == NULL) {
        fprintf(stderr, "Failed to open (%s): %s", dirpath, strerror(errno));
        exit(1);
    }
    struct dirent *ent;
    for(ent = readdir(dh); ent != NULL; ent = readdir(dh)) {
        if (streq(ent->d_name, ".") || streq(ent->d_name, "..")) {
            continue;
        }
        // FIXME:
        std::string tmp = dirpath;
        tmp += "/";
        tmp += ent->d_name;

        instantiate_message_handlers(config);
        ::printf("**************** Analyzing (%s)\n", ent->d_name);
        parse_filepath(tmp.c_str());
        ::printf("**************** End analysis (%s)\n\n", ent->d_name);
        clear_message_handlers();
    }
}

void MAVLink_Reader::parse_filepath(const char *filepath)
{
    int fd = open(filepath, O_RDONLY);
    if (fd == -1) {
        fprintf(stderr, "Failed to open (%s): %s\n", filepath, strerror(errno));
        exit(1);
    }

    examining_filename = filepath;
    return parse_fd(fd);
}

void MAVLink_Reader::parse_fd(int fd)
{
    mavlink_message_t mav_msg;
    mavlink_status_t mav_status;
    char buf[1<<16];
    uint32_t packet_count = 0;
    uint8_t timestamp_offset = 0;
    bool done_timestamp = false;
    uint64_t timestamp = 0;
    while (true) {
        ssize_t bytes_read = read(fd, buf, sizeof(buf));
        if (bytes_read == -1) { 
            fprintf(stderr, "Read failed: %s\n", strerror(errno));
            exit(1);
        }
        if (bytes_read == 0) {
            break;
        }

        for (uint32_t i=0; i<bytes_read; i++) {
            // ::printf("Read (%d)\n", i);
            if (!done_timestamp) {
                timestamp <<= 8;
                timestamp |= (uint8_t)(buf[i]);
                if (timestamp_offset++ == 7) {
                    done_timestamp = true;
                    // ::printf("timestamp (%lu)\n", timestamp);
                    timestamp_offset = 0;
                }
            } else {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &mav_msg, &mav_status)) {
                    // printf("Packet received at %d\n", i);
                    handle_message_received(timestamp, mav_msg);
                    packet_count++;
                    done_timestamp = false;
                    timestamp = 0;
                }
            }
        }
    }

    for(int i=0; i<next_message_handler; i++) {
        message_handler[i]->end_of_log();
    }

    ::fprintf(stderr, "Packet count: %d\n", packet_count);
}


/*
* main - entry point
*/
int main(int argc, char* argv[])
{
    MAVLink_Reader reader;
    reader.parse_arguments(argc, argv);
    reader.run();
}
