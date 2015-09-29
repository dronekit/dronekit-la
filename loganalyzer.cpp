#include "loganalyzer.h"

#include "mavlink_reader.h"

#include "heart.h"

#include <syslog.h>
#include "la-log.h"

#include <sys/stat.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>

#include <regex>

#include "dataflash_reader.h"

#include "analyzing_dataflash_message_handler.h"
#include "analyzing_mavlink_message_handler.h"

void LogAnalyzer::parse_path(const char *path)
{
    if (!strcmp(path, "-")) {
        parse_fd(reader, fileno(stdin));
        reader->clear_message_handlers();
        return;
    }
    
    struct stat buf;
    if (stat(path, &buf) == -1) {
        fprintf(stderr, "Failed to stat (%s): %s\n", path, strerror(errno));
        exit(1);
    }

    switch (buf.st_mode & S_IFMT) {
    case S_IFREG:
        parse_filepath(path);
        return;
    case S_IFDIR:
        return parse_directory_full_of_files(path);
    default:
        fprintf(stderr, "Not a file or directory\n");
        exit(1);
    }
}

void LogAnalyzer::parse_directory_full_of_files(const char *dirpath)
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

        ::printf("**************** Analyzing (%s)\n", ent->d_name);
        parse_filepath(tmp.c_str());
        ::printf("**************** End analysis (%s)\n\n", ent->d_name);
    }
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
void LogAnalyzer::parse_filepath(const char *filepath)
{
    int fd = xopen(filepath, O_RDONLY);

    instantiate_message_handlers();
    parse_fd(reader, fd);
    reader->clear_message_handlers();
}

void LogAnalyzer::instantiate_message_handlers()
{
    if (_vehicle == NULL) { 
        _vehicle = new AnalyzerVehicle::Base();
    }
    Analyze *analyze = new Analyze(_vehicle);
    if (analyze != NULL) {
        analyze->set_output_style(output_style);
        if (_analyzer_names_to_run.size()) {
            analyze->set_analyzer_names_to_run(_analyzer_names_to_run);
        }
        analyze->instantiate_analyzers(_config);
        // which base class doesn't really matter here:
        Analyzing_MAVLink_Message_Handler *handler = new Analyzing_MAVLink_Message_Handler(analyze, _vehicle);
        // Message_Handler *x = static_cast<MAVLink_Message_Handler*>(analyze);
        reader->add_message_handler(handler, "Analyze");
    } else {
        la_log(LOG_ERR, "Failed to create analyze");
        abort();
    }
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
    reader->feed(_client_buf, _client->_buflen_content);
    _client->_buflen_content = 0;
}

void LogAnalyzer::run_live_analysis()
{
    INIReader *config = get_config();

    reader = new MAVLink_Reader(config);

    _client = new Telem_Forwarder_Client(_client_buf, sizeof(_client_buf));
    _client->configure(config);

    Heart *heart= new Heart(_client->fd_telem_forwarder, &_client->sa_tf);
    if (heart != NULL) {
        reader->add_message_handler(heart, "Heart");
    } else {
        la_log(LOG_INFO, "Failed to create heart");
    }

    instantiate_message_handlers();

    select_loop();
}

void LogAnalyzer::run_df(const char *_pathname)
{
    get_config();

    reader = new DataFlash_Reader(_config);

    if (_vehicle == NULL) {
        _vehicle = new AnalyzerVehicle::Base();
    }
    Analyze *analyze = new Analyze(_vehicle);

    if (analyze != NULL) {
        analyze->set_output_style(output_style);
        if (_analyzer_names_to_run.size()) {
            analyze->set_analyzer_names_to_run(_analyzer_names_to_run);
        }
        analyze->instantiate_analyzers(_config);
        Analyzing_DataFlash_Message_Handler *handler = new Analyzing_DataFlash_Message_Handler(analyze, _vehicle);
        // Message_Handler *x = static_cast<DataFlash_Message_Handler*>(handler);
        reader->add_message_handler(handler, "Analyze");
    } else {
        la_log(LOG_ERR, "Failed to create analyze");
        abort();
    }

    int fd = xopen(_pathname, O_RDONLY);

    parse_fd(reader, fd);
    reader->clear_message_handlers();
    exit(0);
}

void LogAnalyzer::show_version_information()
{
    ::printf("Version: " GIT_VERSION "\n");
}

void LogAnalyzer::list_analyzers()
{
    Analyze *analyze = new Analyze(_vehicle);

    analyze->instantiate_analyzers(_config);
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

void LogAnalyzer::run()
{
    // la_log(LOG_INFO, "loganalyzer starting: built " __DATE__ " " __TIME__);
    // signal(SIGHUP, sighup_handler);

    if (_show_version_information) {
        show_version_information();
        exit(0);
    }
    if (_do_list_analyzers) {
        list_analyzers();
        exit(0);
    }

    expand_names_to_run();

    if (_model_string != NULL) {
        if (streq(_model_string,"copter")) {
            _vehicle = new AnalyzerVehicle::Copter();
//            _analyze->set_vehicle_copter();
            if (_frame_string != NULL) {
                ((AnalyzerVehicle::Copter*)_vehicle)->set_frame(_frame_string);
            }
        // } else if (streq(model_string,"plane")) {
        //     model = new AnalyzerVehicle::Plane();
        // } else if (streq(model_string,"rover")) {
        //     model = new AnalyzerVehicle::Rover();
        } else {
            la_log(LOG_ERR, "Unknown model type (%s)", _model_string);
            exit(1);
        }
    }

    if (_use_telem_forwarder) {
        return run_live_analysis();
    }

    if (_pathname == NULL) {
        usage();
        exit(1);
    }

    output_style = Analyze::OUTPUT_JSON;
    if (output_style_string != NULL) {
        output_style = Analyze::OUTPUT_JSON;
        if (streq(output_style_string, "json")) {
            output_style = Analyze::OUTPUT_JSON;
        } else if(streq(output_style_string, "plain-text")) {
            output_style = Analyze::OUTPUT_PLAINTEXT;
        } else if(streq(output_style_string, "brief")) {
            output_style = Analyze::OUTPUT_BRIEF;
        } else if(streq(output_style_string, "html")) {
            output_style = Analyze::OUTPUT_HTML;
        } else {
            usage();
            exit(1);
        }
    }

    if (strstr(_pathname, ".BIN") ||
        strstr(_pathname, ".bin")) {
        return run_df(_pathname);
    }
    
    INIReader *config = get_config();

    reader = new MAVLink_Reader(config);
    ((MAVLink_Reader*)reader)->set_is_tlog(true);

    return parse_path(_pathname);
}

void LogAnalyzer::usage()
{
    ::printf("Usage:\n");
    ::printf("%s [OPTION] [FILE]\n", program_name());
    ::printf(" -c filepath      use config file filepath\n");
    ::printf(" -t               connect to telem forwarder to receive data\n");
    ::printf(" -m modeltype     override model; copter|plane|rover\n");
    ::printf(" -f frame         set frame; QUAD|Y6\n");
    ::printf(" -s style         use output style (plain-text|json)\n");
    ::printf(" -h               display usage information\n");
    ::printf(" -l               list analyzers\n");
    ::printf(" -a               specify analzers to run (comma-separated list)\n");
    ::printf(" -V               display version information\n");
    ::printf("\n");
    ::printf("Example: %s -s json 1.solo.tlog\n", program_name());
    ::printf("Example: %s -a \"Ever Flew, Battery\" 1.solo.tlog\n", program_name());
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

    while ((opt = getopt(argc, argv, "hc:ts:m:f:Vla:")) != -1) {
        switch(opt) {
        case 'h':
            usage();
            break;
        case 'c':
            config_filename = optarg;
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
        _pathname = argv[optind++];
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
