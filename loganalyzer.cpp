#include "loganalyzer.h"

#include "mavlink_reader.h"

#include "heart.h"

#include <iostream> // for cout
#include <syslog.h>
#include "la-log.h"

#include <sys/stat.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>

#include <regex>

#include "dataflash_reader.h"
#include "dataflash_textdump_reader.h"

#include "analyzing_dataflash_message_handler.h"
#include "analyzing_mavlink_message_handler.h"

void LogAnalyzer::parse_path(const char *path)
{
    create_vehicle_from_commandline_arguments();
    if (_vehicle == NULL) {
        _vehicle = new AnalyzerVehicle::Base();
    }

    bool do_stdin = false;

    log_format_t log_format = log_format_none;
    if (!strcmp(path, "-")) {
        do_stdin = true;
    } else if (strstr(path, ".BIN") ||
               strstr(path, ".bin")) {
        log_format = log_format_df;
    } else if (strstr(path, ".log") ||
               strstr(path, ".LOG")) {
        log_format = log_format_log;
    } else if (strstr(path, ".tlog") ||
               strstr(path, ".TLOG")) {
        log_format = log_format_tlog;
    }

    if (force_format() != log_format_none) {
        log_format = force_format();
    }
    
    if (log_format == log_format_none) {
        if (do_stdin) {
            ::fprintf(stderr, "You asked to parse stdin but did not force a format type\n");
        } else {
            ::fprintf(stderr, "Unable to determine log type from filename (%s); try -i?\n", path);
        }
        usage();
        exit(1);
    }

    switch (log_format) {
    case log_format_tlog:
        prep_for_tlog();
        break;
    case log_format_df:
        prep_for_df();
        break;
    case log_format_log:
        prep_for_log();
        break;
    default:
        abort();
    }

    int fd;
    if (streq(path, "-")) {
        fd = fileno(stdin);
    } else {
        fd = xopen(path, O_RDONLY);
    }

    if (output_style == Analyze::OUTPUT_BRIEF) {
        printf("%25s: ", path);
    }

    parse_fd(reader, fd);

    if (!streq(path, "-")) {
        close(fd);
    }

    if (output_style == Analyze::OUTPUT_BRIEF) {
        printf("\n");
    }

    switch (log_format) {
    case log_format_tlog:
        cleanup_after_tlog();
        break;
    case log_format_df:
        cleanup_after_df();
        break;
    case log_format_log:
        cleanup_after_log();
        break;
    default:
        abort();
    }
    
    delete _vehicle;
    _vehicle = NULL;
}

int LogAnalyzer::xopen(const char *filepath, const uint8_t mode)
{
    int fd = open(filepath, mode);
    if (fd == -1) {
        fprintf(stderr, "Failed to open (%s): %s\n", filepath, strerror(errno));
        exit(1);
    }
    return fd;
}

void LogAnalyzer::prep_for_tlog()
{
    reader = new MAVLink_Reader(config());
    ((MAVLink_Reader*)reader)->set_is_tlog(true);

    Analyze *analyze = create_analyze();

    Analyzing_MAVLink_Message_Handler *handler = new Analyzing_MAVLink_Message_Handler(analyze, _vehicle);
    reader->add_message_handler(handler, "Analyze");
}

void LogAnalyzer::cleanup_after_tlog()
{
    reader->clear_message_handlers();
}
void LogAnalyzer::cleanup_after_df()
{
    reader->clear_message_handlers();
}
void LogAnalyzer::cleanup_after_log()
{
    reader->clear_message_handlers();
}

void LogAnalyzer::do_idle_callbacks()
{
    reader->do_idle_callbacks();
}
void LogAnalyzer::pack_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds)
{
    _client->pack_select_fds(fds_read, fds_write, fds_err, nfds);
}

void LogAnalyzer::handle_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds)
{
    _client->handle_select_fds(fds_read, fds_write, fds_err, nfds);

    // FIXME: find a more interesting way of doing this...
    reader->feed(_client->_recv_buf, _client->_recv_buflen_content);
    _client->_recv_buflen_content = 0;
}

void LogAnalyzer::run_live_analysis()
{
    reader = new MAVLink_Reader(config());

    _client = new Telem_Forwarder_Client(_client_recv_buf, sizeof(_client_recv_buf));
    _client->configure(config());

    writer = new MAVLink_Writer(config());
    if (writer == NULL) {
        la_log(LOG_ERR, "Failed to create writer from (%s)\n", config_filename);
        exit(1);
    }

    Heart *heart= new Heart(writer);
    if (heart != NULL) {
        reader->add_message_handler(heart, "Heart");
    } else {
        la_log(LOG_INFO, "Failed to create heart");
    }

    create_vehicle_from_commandline_arguments();
    if (_vehicle == NULL) {
        _vehicle = new AnalyzerVehicle::Base();
    }

    Analyze *analyze = create_analyze();

    Analyzing_MAVLink_Message_Handler *handler = new Analyzing_MAVLink_Message_Handler(analyze, _vehicle);
    reader->add_message_handler(handler, "Analyze");

    select_loop();

    _vehicle = NULL; // leak this memory
}

Analyze *LogAnalyzer::create_analyze()
{
    Analyze *analyze = new Analyze(_vehicle);
    if (analyze == NULL) {
        la_log(LOG_ERR, "Failed to create analyze");
        abort();
    }

    analyze->set_output_style(output_style);
    if (_analyzer_names_to_run.size()) {
        analyze->set_analyzer_names_to_run(_analyzer_names_to_run);
    }


    analyze->set_pure_output(_pure_output);

    analyze->instantiate_analyzers(config());

    return analyze;
}

void LogAnalyzer::prep_for_df()
{
    reader = new DataFlash_Reader(config());

    Analyze *analyze = create_analyze();

    Analyzing_DataFlash_Message_Handler *handler = new Analyzing_DataFlash_Message_Handler(analyze, _vehicle);
    reader->add_message_handler(handler, "Analyze");
}

void LogAnalyzer::prep_for_log()
{
    reader = new DataFlash_TextDump_Reader(config());

    Analyze *analyze = create_analyze();

    Analyzing_DataFlash_Message_Handler *handler = new Analyzing_DataFlash_Message_Handler(analyze, _vehicle);
    reader->add_message_handler(handler, "Analyze");
}

void LogAnalyzer::show_version_information()
{
    ::printf("Version: " DRONEKIT_LA_VERSION "\n");
    ::printf("Git-Version: " GIT_VERSION "\n");
}

void LogAnalyzer::list_analyzers()
{
    Analyze *analyze = new Analyze(_vehicle);

    analyze->instantiate_analyzers(config());
    std::vector<Analyzer *> analyzers = analyze->analyzers();
    for (std::vector<Analyzer*>::iterator it = analyzers.begin();
         it != analyzers.end();
         ++it) {
        ::printf("%s\n", (*it)->name().c_str());
    }
}

// From: http://stackoverflow.com/questions/236129/split-a-string-in-c
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}
std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}
// thanks stackoverflow!

void LogAnalyzer::expand_names_to_run()
{
    std::vector<std::string> new_names;
    for (std::vector<std::string>::const_iterator it = _analyzer_names_to_run.begin();
         it != _analyzer_names_to_run.end();
         ++it) {
        // insert lambda here if you dare.
        std::vector<std::string> tmp = split((*it),',');
        
        for (std::vector<std::string>::const_iterator iy = tmp.begin();
             iy != tmp.end();
             ++iy) {
            std::string value = std::regex_replace(*iy, std::regex("^ +| +$|( ) +"), "$1");
            new_names.push_back(value);
        }
    }
    _analyzer_names_to_run = new_names;
}

void LogAnalyzer::create_vehicle_from_commandline_arguments()
{
    if (_model_string != NULL) {
        if (strieq(_model_string,"copter")) {
            _vehicle = new AnalyzerVehicle::Copter();
//            _analyze->set_vehicle_copter();
            if (_frame_string != NULL) {
                ((AnalyzerVehicle::Copter*)_vehicle)->set_frame(_frame_string);
            }
        } else if (strieq(_model_string,"plane")) {
            _vehicle = new AnalyzerVehicle::Plane();
        // } else if (streq(model_string,"rover")) {
        //     model = new AnalyzerVehicle::Rover();
        } else {
            la_log(LOG_ERR, "Unknown model type (%s)", _model_string);
            exit(1);
        }
        _vehicle->set_vehicletype_is_forced(true);
    } else if (_frame_string != NULL) {
        la_log(LOG_ERR, "Can not specify frame type without specifying model type");
        exit(1);
    }
}

void LogAnalyzer::run()
{
    // la_log(LOG_INFO, "loganalyzer starting: built " __DATE__ " " __TIME__);
    // signal(SIGHUP, sighup_handler);
    init_config();

    if (_show_version_information) {
        show_version_information();
        exit(0);
    }
    if (_do_list_analyzers) {
        list_analyzers();
        exit(0);
    }

    expand_names_to_run();

    if (_use_telem_forwarder) {
        return run_live_analysis();
    }

    if (_paths == NULL) {
        usage();
        exit(1);
    }

    output_style = Analyze::OUTPUT_JSON;
    if (output_style_string != NULL) {
        output_style = Analyze::OUTPUT_JSON;
        if (strieq(output_style_string, "json")) {
            output_style = Analyze::OUTPUT_JSON;
        } else if(strieq(output_style_string, "plain-text")) {
            output_style = Analyze::OUTPUT_PLAINTEXT;
        } else if(strieq(output_style_string, "brief")) {
            output_style = Analyze::OUTPUT_BRIEF;
        } else if(strieq(output_style_string, "html")) {
            output_style = Analyze::OUTPUT_HTML;
        } else {
            usage();
            exit(1);
        }
    }

    if (forced_format_string != NULL) {
        if (strieq(forced_format_string, "tlog")) {
            _force_format = log_format_tlog;
        } else if(strieq(forced_format_string, "df")) {
            _force_format = log_format_df;
        } else if(strieq(forced_format_string, "log")) {
            _force_format = log_format_log;
        } else {
            usage();
            exit(1);
        }
    }

    for (uint8_t i=0; i<_pathcount; i++) {
        parse_path(_paths[i]);
    }
}

void LogAnalyzer::usage()
{
    ::printf("Usage:\n");
    ::printf("%s [OPTION] [FILE...]\n", program_name());
    ::printf(" -c FILEPATH      use config file filepath\n");
    // ::printf(" -t               connect to telem forwarder to receive data\n");
    ::printf(" -m MODELTYPE     override model; copter|plane|rover\n");
    ::printf(" -f FRAME         set frame; QUAD|Y6\n");
    ::printf(" -s STYLE         use output style (plain-text|json|brief)\n");
    ::printf(" -h               display usage information\n");
    ::printf(" -l               list analyzers\n");
    ::printf(" -a               specify analyzers to run (comma-separated list)\n");
    ::printf(" -i FORMAT        specify input format (tlog|df|log)\n");
    ::printf(" -p               pure output - no deprecated fields\n");
    ::printf(" -V               display version information\n");
    ::printf("\n");
    ::printf("Example: %s -s json 1.solo.tlog\n", program_name());
    ::printf("Example: %s -a \"Ever Flew, Battery\" 1.solo.tlog\n", program_name());
    ::printf("Example: %s -s brief 1.solo.tlog 2.solo.tlog logs/*.tlog\n", program_name());
    ::printf("Example: %s - (analyze stdin)\n", program_name());
    ::printf("Example: %s x.log (analyze text-dumped dataflash log)\n", program_name());
    exit(0);
}
const char *LogAnalyzer::program_name()
{
    if (_argv == NULL) {
        return "[Unknown]";
    }
    return _argv[0];
}


void LogAnalyzer::parse_arguments(int argc, char *argv[])
{
    int opt;
    _argc = argc;
    _argv = argv;

    while ((opt = getopt(argc, argv, "hc:ts:m:pf:Vla:i:")) != -1) {
        switch(opt) {
        case 'h':
            usage();
            break;
        case 'c':
            config_filename = optarg;
            break;
        case 'i':
            forced_format_string = optarg;
            break;
        case 't':
            _use_telem_forwarder = true;
            break;
        case 's':
            output_style_string = optarg;
            break;
        case 'm':
            _model_string = optarg;
            break;
        case 'p':
            _pure_output = true;
            break;
        case 'f':
            _frame_string = optarg;
            break;
        case 'V':
            _show_version_information = true;
            break;
        case 'l':
            _do_list_analyzers = true;
            break;
        case 'a':
            _analyzer_names_to_run.push_back(optarg);
            break;
        }
    }
    if (optind < argc) {
        _paths = &argv[optind];
        _pathcount = argc-optind;
    }
}

/*
* main - entry point
*/
int main(int argc, char* argv[])
{
    LogAnalyzer analyzer;
    analyzer.parse_arguments(argc, argv);
    analyzer.run();
    exit(0);
}
