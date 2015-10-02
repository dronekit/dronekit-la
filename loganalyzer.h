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
        Common_Tool()
        { }

    void tlog_parse_path(const char *path);
    void tlog_parse_filepath(const char *filepath);
    void tlog_parse_open_filepath(const char *filepath, int fd);
    void run();

    void parse_arguments(int argc, char *argv[]);

private:
    bool _use_telem_forwarder = false;
    Telem_Forwarder_Client *_client = NULL;
    uint8_t _client_buf[512]; // FIXME constant was TELEM_PKT_MAX

    char **_paths = NULL;
    uint8_t _pathcount;

    Format_Reader *reader;
    Analyze::output_style_option output_style = Analyze::OUTPUT_JSON;
    long _argc;
    char **_argv;
    const char * output_style_string = NULL;
    const char * _model_string = NULL;
    const char * _frame_string = NULL;

    AnalyzerVehicle::Base *_vehicle = NULL;
    void create_vehicle_from_commandline_arguments();

    const char *program_name();

    void usage();
    void instantiate_message_handlers();

    void run_live_analysis();
    void run_df(const char *_pathname);
    void do_idle_callbacks();

    void pack_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds);
    void handle_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds);

    int xopen(const char *filepath, uint8_t mode);

    void show_version_information();
    bool _show_version_information = false;
    void list_analyzers();
    bool _do_list_analyzers = false;

    void expand_names_to_run();
    std::vector<std::string> _analyzer_names_to_run;
};

#endif
