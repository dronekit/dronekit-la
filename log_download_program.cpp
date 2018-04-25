#include "log_download_program.h"

#include "log_download.h"
#include "log_list.h"

#include "heart.h"
#include "mavlink_reader.h"
#include "telem_forwarder_client.h"
#include "telem_serial.h"

#include "la-log.h"

#include <signal.h>
#include <unistd.h>

const char *Log_Download_Program::program_name()
{
    if (_argv == NULL) {
        return "[Unknown]";
    }
    return _argv[0];
}

void Log_Download_Program::usage()
{
    ::printf("Usage:\n");
    ::printf("%s [OPTION] LOGNUM...\n", program_name());
    ::printf(" -c filepath      use config file filepath\n");
    ::printf(" -h               display usage information\n");
    ::printf(" -d               debug mode\n");
    ::printf(" -s PORT          use serial port\n");
    ::printf(" -l               list available logs\n");
    ::printf("\n");
    ::printf("Example: %s\n", program_name());
    exit(0);
}

Log_Download_Program logger;

void sighup_handler(int signal)
{
    logger.sighup_handler(signal);
}

void LD_Client::do_idle_callbacks()
{
    reader->do_idle_callbacks();
}
void LD_Client::sighup_handler()
{
    reader->sighup_handler();
}
void LD_Client::pack_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds)
{
    telem_client->pack_select_fds(fds_read, fds_write, fds_err, nfds);
}
void LD_Client::do_writer_sends()
{
    telem_client->do_writer_sends();
}
void LD_Client::handle_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds) {
    telem_client->handle_select_fds(fds_read, fds_write, fds_err, nfds);

    // FIXME: find a more interesting way of doing this...  we should
    // probably rejig things so that the client is a mavlink_reader
    // and simply produces mavlink_message_t's itself, rather than us
    // handing off the a dedicated parser object here.
    reader->feed(telem_client->_recv_buf, telem_client->_recv_buflen_content);
    telem_client->_recv_buflen_content = 0;
}


void Log_Download_Program::do_idle_callbacks()
{
    client->do_idle_callbacks();
}

void Log_Download_Program::sighup_received_tophalf()
{
    client->sighup_handler();
}

uint32_t Log_Download_Program::select_timeout_us() {
    if (_writer->any_data_to_send()) {
        return 0;
    }
    return Common_Tool::select_timeout_us();
}

void Log_Download_Program::pack_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds)
{
    client->pack_select_fds(fds_read, fds_write, fds_err, nfds);
}

void Log_Download_Program::handle_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds)
{
    client->handle_select_fds(fds_read, fds_write, fds_err, nfds);

    // handle data *to* e.g. telem_forwarder
    client->do_writer_sends();
}

void Log_Download_Program::create_serial_client()
{
    client = new LD_Client();
    client->reader = new MAVLink_Reader(config());
    if (client->reader == NULL) {
        la_log(LOG_ERR, "Failed to create reader from (%s)\n", config_filename);
        exit(1);
    }

    Telem_Serial *x = new Telem_Serial();
    client->telem_client = x;

//    client->telem_client->configure(config());
    ::fprintf(stderr, "path=%s baud=%u\n", serial_port_path, serial_port_baud);
    x->set_serial(serial_port_path, serial_port_baud, serial_port_flow);
    x->init();
}

void Log_Download_Program::create_telem_forwarder_client()
{
    client = new LD_Client();
    client->reader = new MAVLink_Reader(config());
    if (client->reader == NULL) {
        la_log(LOG_ERR, "Failed to create reader from (%s)\n", config_filename);
        exit(1);
    }

    Telem_Forwarder_Client *x = new Telem_Forwarder_Client();
    client->telem_client = x;

    // client->telem_client->configure(config());
    x->set_address("127.0.0.1", 14552);

    x->init();
}

void Log_Download_Program::run()
{
    init_config();

    if (! debug_mode) {
        la_log_syslog_open();
    }

    la_log(LOG_INFO, "dataflash_logger starting: built " __DATE__ " " __TIME__);
    signal(SIGHUP, ::sighup_handler);

    if (serial_port) {
        create_serial_client();
    } else {
        create_telem_forwarder_client();
    }

    _writer = new MAVLink_Writer(config());
    if (_writer == NULL) {
        la_log(LOG_ERR, "Failed to create writer from (%s)\n", config_filename);
        exit(1);
    }
    _writer->add_client(client->telem_client);

    // instantiate message handlers:
    if (list) {
        Log_List *log_list = new Log_List(_writer);
        if (log_list != NULL) {
            client->reader->add_message_handler(log_list, "Log_List");
        } else {
            la_log(LOG_INFO, "Failed to create log_list");
        }
    } else {
        Log_Download *log_download = new Log_Download(_writer, logs_to_download);
        if (log_download != NULL) {
            client->reader->add_message_handler(log_download, "Log_Download");
        } else {
            la_log(LOG_INFO, "Failed to create log_download");
        }
    }

    Heart *heart = new Heart(_writer);
    if (heart != NULL) {
        client->reader->add_message_handler(heart, "Heart");
    } else {
        la_log(LOG_INFO, "Failed to create heart");
    }

    return select_loop();
}


void Log_Download_Program::parse_arguments(int argc, char *argv[])
{
    int opt;
    _argc = argc;
    _argv = argv;

    char *tmp_serial_port_path = nullptr;
    while ((opt = getopt(argc, argv, "hc:ds:l")) != -1) {
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
        case 's':
            serial_port = true;
            tmp_serial_port_path = optarg;
            break;
        case 'l':
            list = true;
            break;
        }
    }

    if (tmp_serial_port_path != nullptr) {
        serial_port_path = strdup(tmp_serial_port_path);
        char *x = strstr(serial_port_path, ":");
        if (x != nullptr) {
            *x = '\0';
            x++;
            serial_port_baud = atoi(x);
            if (serial_port_baud == 0) {
                ::fprintf(stderr, "Failed to parse baudrate");
                abort();
            }
            ::fprintf(stderr, "baud: %u\n", serial_port_baud);
        }
    }

    for (auto index = optind; index < argc; index++) {
        uint16_t n = atoi(argv[index]);
        if (n == 0) {
            ::fprintf(stderr, "Failed to parse (%s)", argv[index]);
            abort();
        }
        logs_to_download.push_back(n);
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
