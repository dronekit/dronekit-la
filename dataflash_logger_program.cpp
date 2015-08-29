#include "dataflash_logger_program.h"

#include "dataflash_logger.h"
#include "heart.h"
#include "mavlink_reader.h"

#include "la-log.h"

#include <signal.h>
#include <unistd.h>

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
    sighup_received = true;
}
    
void DataFlash_Logger_Program::loop()
{
    while (1) {
	if (sighup_received) {
            reader->sighup_handler();
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
        FD_SET(client->fd_telem_forwarder, &fds);
        if (client->fd_telem_forwarder >= nfds)
            nfds = client->fd_telem_forwarder + 1;
	fds_err = fds;

        timeout.tv_sec = 0;
        timeout.tv_usec = 200000;
        int res = select(nfds, &fds, NULL, &fds_err, &timeout);

        if (res < 0) {
            unsigned skipped = 0;
            // if ((skipped = can_log_error()) >= 0)
            la_log(LOG_ERR, "[%u] select: %s", skipped, strerror(errno));
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
        if (FD_ISSET(client->fd_telem_forwarder, &fds_err)) {
            unsigned skipped = 0;
            // if ((skipped = can_log_error()) >= 0)
                la_log(LOG_ERR, "[%u] select(fd_telem_forwarder): %s", skipped, strerror(errno));
	}

        if (FD_ISSET(client->fd_telem_forwarder, &fds)) {
   	    uint32_t len = client->handle_recv();
            ::fprintf(stderr, "feeding %u bytes\n", len);
            reader->feed(_buf, len);
        }

	reader->do_idle_callbacks();
    } /* while (1) */
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

    client = new Telem_Forwarder_Client(_buf, sizeof(_buf));
    client->configure(config);

    instantiate_message_handlers(config, client->fd_telem_forwarder, &client->sa_tf);
    return loop();
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
