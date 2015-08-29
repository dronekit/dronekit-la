#include "common_tool.h"

#include "mavlink_reader.h"
#include "telem_forwarder_client.h"

class DataFlash_Logger_Program : Common_Tool {
public:
    DataFlash_Logger_Program() :
        Common_Tool(),
        use_telem_forwarder(false),
        _argc(0),
        _argv(NULL),
        _buf{ },
        sighup_received(false)
        { }
    void instantiate_message_handlers(INIReader *config,
                                      int fd_telem_forwarder,
                                      struct sockaddr_in *sa_tf);
    void sighup_handler(int signal);
    void run();
    void parse_arguments(int argc, char *argv[]);
    const char *program_name();

    void loop();
    
private:
    void usage();

    bool use_telem_forwarder;
    MAVLink_Reader *reader;

    long _argc;
    char **_argv;

    uint8_t _buf[512]; // FIXME constant was TELEM_PKT_MAX

    bool sighup_received;

    Telem_Forwarder_Client *client;
};
