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

void DataFlash_Logger_Program::do_idle_callbacks()
{
    reader->do_idle_callbacks();
}

void DataFlash_Logger_Program::sighup_received_tophalf()
{
    reader->sighup_handler();
}

void DataFlash_Logger_Program::pack_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds)
{
    client->pack_select_fds(fds_read, fds_write, fds_err, nfds);
}

void DataFlash_Logger_Program::handle_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds)
{
    client->handle_select_fds(fds_read, fds_write, fds_err, nfds);

    // FIXME: find a more interesting way of doing this...
    reader->feed(_buf, client->_buflen_content);
    client->_buflen_content = 0;
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
    return select_loop();
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
