#include "common_tool.h"

#include "mavlink_reader.h"
#include "telem_forwarder_client.h"

class DataFlash_Logger_Program : public Common_Tool {
public:
    DataFlash_Logger_Program() :
        Common_Tool()
        { }
    void instantiate_message_handlers(INIReader *config,
                                      int fd_telem_forwarder,
                                      struct sockaddr_in *sa_tf);
    void run();
    void parse_arguments(int argc, char *argv[]);
    const char *program_name();

    void pack_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds) override;
    void handle_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds) override;
    
private:
    void usage();
    void sighup_received_tophalf() override;
    void do_idle_callbacks() override;

    MAVLink_Reader *reader;

    long _argc = 0;
    char **_argv = NULL;

    uint8_t _buf[512] = { }; // FIXME constant was TELEM_PKT_MAX

    Telem_Forwarder_Client *client = NULL;
};
