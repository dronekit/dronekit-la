#ifndef LOGANALYZER_H
#define LOGANALYZER_H

/*
 * loganalyzer
 *
 * Receive telemetry (mavlink) via UDP from Solo, and create dataflash log files
 *
 * Initiate a remote-dataflash stream
 */

#include "INIReader.h"

#include "mavlink_message_handler.h"
#include "mavlink_reader.h"
#include "telem_forwarder_client.h"

#include "analyze.h"
#include "common_tool.h"

class LogAnalyzer : Common_Tool {

public:
    LogAnalyzer() :
        Common_Tool(),
        _use_telem_forwarder(false),
        _client(NULL),
        _pathname(NULL),
        output_style(Analyze::OUTPUT_JSON),
        output_style_string(NULL),
        _model_string(NULL),
        _vehicle(NULL)
        { }

    void parse_path(const char *path);
    void parse_filepath(const char *filepath);
    void parse_directory_full_of_files(const char *dirpath);
    void run();

    void parse_arguments(int argc, char *argv[]);

private:
    bool _use_telem_forwarder;
    Telem_Forwarder_Client *_client;
    uint8_t _client_buf[512]; // FIXME constant was TELEM_PKT_MAX

    char *_pathname;
    MAVLink_Reader *reader;
    Analyze::output_style_option output_style;
    long _argc;
    char **_argv;
    const char * output_style_string;
    const char * _model_string;

    AnalyzerVehicle::Base *_vehicle;

    const char *program_name();

    void usage();
    void instantiate_message_handlers();

    void live_analysis();
    void do_idle_callbacks();
    void pack_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds);
    void handle_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds);
};

#endif
