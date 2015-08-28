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
