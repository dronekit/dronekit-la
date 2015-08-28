#include "common_tool.h"

#include "mavlink_reader.h"

class DataFlash_Logger_Program : Common_Tool {
public:
    DataFlash_Logger_Program() :
        Common_Tool(),
        use_telem_forwarder(false),
        _argc(0),
        _argv(NULL)
        { }
    void instantiate_message_handlers(INIReader *config,
                                      int fd_telem_forwarder,
                                      struct sockaddr_in *sa_tf);
    void sighup_handler(int signal);
    void run();
    void parse_arguments(int argc, char *argv[]);
    const char *program_name();

private:
    void usage();

    bool use_telem_forwarder;
    MAVLink_Reader *reader;

    long _argc;
    char **_argv;
};
