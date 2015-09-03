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

#include "analyze.h"
#include "common_tool.h"

class LogAnalyzer : Common_Tool {

public:
    LogAnalyzer() :
        Common_Tool(),
        _pathname(NULL),
        output_style(Analyze::OUTPUT_JSON),
        output_style_string(NULL)
        { }

    void parse_path(const char *path);
    void parse_filepath(const char *filepath);
    void parse_directory_full_of_files(const char *dirpath);
    void run();

    void parse_arguments(int argc, char *argv[]);

private:
    char *_pathname;
    MAVLink_Reader *reader;
    Analyze::output_style_option output_style;
    long _argc;
    char **_argv;
    const char * output_style_string;
    const char *program_name();

    void usage();
    void instantiate_message_handlers();
};

#endif
