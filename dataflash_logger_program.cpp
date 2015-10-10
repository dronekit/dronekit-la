#include "dataflash_logger_program.h"

#include "dataflash_logger.h"
#include "heart.h"
#include "mavlink_reader.h"
#include "telem_forwarder_client.h"
#include "telem_serial.h"

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
    ::printf(" -h               display usage information\n");
    ::printf(" -d               debug mode\n");
    ::printf("\n");
    ::printf("Example: %s\n", program_name());
    exit(0);
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

uint32_t DataFlash_Logger_Program::select_timeout_us() {
    if (_writer->buf_used()) {
        return 0;
    }
    return Common_Tool::select_timeout_us();
}

void DataFlash_Logger_Program::pack_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds)
{
    client->pack_select_fds(fds_read, fds_write, fds_err, nfds);
}

// FIXME: move this up?!
void DataFlash_Logger_Program::do_writer_sends()
{
    while (_writer_buflen_start != _writer_buflen_stop) { // FIXME: use file descriptors!
        bool tail_first = false;
        if (_writer_buflen_stop < _writer_buflen_start) {
            tail_first = true;
        }
        uint32_t bytes_to_send = tail_first ? (_writer_buflen - _writer_buflen_start) : (_writer_buflen_stop - _writer_buflen_start);

        int32_t sent = client->write((const char *)&_writer_buf[_writer_buflen_start], bytes_to_send);
        if (sent < 0) {
            // cry
            break;
        } else if (sent == 0) {
            break;
        } else {
            _writer_buflen_start += sent;
            if (_writer_buflen_start == _writer_buflen) {
                _writer_buflen_start = 0;
            }
        }
    }
}

void DataFlash_Logger_Program::handle_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds)
{
    client->handle_select_fds(fds_read, fds_write, fds_err, nfds);

    // FIXME: find a more interesting way of doing this...
    // handle data *from* e.g. telem_forwarder
    reader->feed(_buf, client->_buflen_content);
    client->_buflen_content = 0;

    // handle data *to* e.g. telem_forwarder
    do_writer_sends();
}

void DataFlash_Logger_Program::run()
{
    init_config();

    if (! debug_mode) {
        la_log_syslog_open();
    }

    la_log(LOG_INFO, "dataflash_logger starting: built " __DATE__ " " __TIME__);
    signal(SIGHUP, ::sighup_handler);

    reader = new MAVLink_Reader(config());
    if (reader == NULL) {
        la_log(LOG_ERR, "Failed to create reader from (%s)\n", config_filename);
        exit(1);
    }

    client = new Telem_Forwarder_Client(_buf, sizeof(_buf));
    // client = new Telem_Serial(_buf, sizeof(_buf));
    client->configure(config());
    client->init();

    _writer = new MAVLink_Writer(config(), _writer_buf, _writer_buflen, _writer_buflen_start, _writer_buflen_stop);
    if (_writer == NULL) {
        la_log(LOG_ERR, "Failed to create writer from (%s)\n", config_filename);
        exit(1);
    }
    
    // instantiate message handlers:
    DataFlash_Logger *dataflash_logger = new DataFlash_Logger(_writer);
    if (dataflash_logger != NULL) {
        reader->add_message_handler(dataflash_logger, "DataFlash_Logger");
    } else {
        la_log(LOG_INFO, "Failed to create dataflash logger");
    }

    Heart *heart= new Heart(_writer);
    if (heart != NULL) {
        reader->add_message_handler(heart, "Heart");
    } else {
        la_log(LOG_INFO, "Failed to create heart");
    }

    return select_loop();
}


void DataFlash_Logger_Program::parse_arguments(int argc, char *argv[])
{
    int opt;
    _argc = argc;
    _argv = argv;

    while ((opt = getopt(argc, argv, "hc:d")) != -1) {
        switch(opt) {
        case 'h':
            usage();
            break;
        case 'c':
            config_filename = optarg;
            break;
        case 'd':
            debug_mode = true;
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
