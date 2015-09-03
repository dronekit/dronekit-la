#include "loganalyzer.h"

#include "mavlink_reader.h"

#include <syslog.h>
#include "la-log.h"

#include <sys/stat.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>

const char *examining_filename; // REMOVE ME!

void LogAnalyzer::parse_path(const char *path)
{
    if (!strcmp(path, "-")) {
        reader->parse_fd(fileno(stdin));
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

void LogAnalyzer::parse_filepath(const char *filepath)
{
    int fd = open(filepath, O_RDONLY);
    if (fd == -1) {
        fprintf(stderr, "Failed to open (%s): %s\n", filepath, strerror(errno));
        exit(1);
    }

    examining_filename = filepath;
    instantiate_message_handlers();
    reader->parse_fd(fd);
    reader->clear_message_handlers();
}

void LogAnalyzer::instantiate_message_handlers()
{
    // if (use_telem_forwarder) {
    //     Heart *heart= new Heart(fd_telem_forwarder, sa_tf);
    //     if (heart != NULL) {
    //         configure_message_handler(config, heart, "Heart");
    //     } else {
    //         la_log(LOG_INFO, "Failed to create heart");
    //     }
    // }

    Analyze *analyze = new Analyze();
    if (analyze != NULL) {
        analyze->set_output_style(output_style);
        analyze->instantiate_analyzers(_config);
        reader->add_message_handler(analyze, "Analyze");
    } else {
        la_log(LOG_ERR, "Failed to create analyze");
        abort();
    }
}

void LogAnalyzer::run()
{
    la_log(LOG_INFO, "loganalyzer starting: built " __DATE__ " " __TIME__);
    // signal(SIGHUP, sighup_handler);

    INIReader *config = get_config();

    reader = new MAVLink_Reader(config);
    reader->set_is_tlog(true);

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
        } else if(streq(output_style_string, "html")) {
            output_style = Analyze::OUTPUT_HTML;
        } else {
            usage();
            exit(1);
        }
    }

    /* prepare sockaddr used to contact telem_forwarder */
    // reader.telem_forwarder_pack_sockaddr(config);

    /* Prepare a port to receive and send data to/from telem_forwarder */
    /* does not return on failure */
    // reader.create_and_bind();

    return parse_path(_pathname);
}

void LogAnalyzer::usage()
{
    ::printf("Usage:\n");
    ::printf("%s [OPTION] [FILE]\n", program_name());
    ::printf(" -c filepath      use config file filepath\n");
    // ::printf(" -t               connect to telem forwarder to receive data\n");
    ::printf(" -s style         use output style (plain-text|json)\n");
    ::printf(" -h               display usage information\n");
    ::printf("\n");
    ::printf("Example: ./dataflash_logger -c /dev/null -s json 1.solo.tlog\n");
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

    while ((opt = getopt(argc, argv, "hc:ts:")) != -1) {
        switch(opt) {
        case 'h':
            usage();
            break;
        case 'c':
            config_filename = optarg;
            break;
        // case 't':
        //     use_telem_forwarder = true;
        //     break;
        case 's':
            output_style_string = optarg;
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
}
