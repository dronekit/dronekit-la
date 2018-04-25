#include "common_tool.h"

#include "mavlink_reader.h"
#include "mavlink_writer.h"
#include "telem_client.h"

class LD_Client {
public:
    void do_idle_callbacks();
    void sighup_handler();
    void pack_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds);
    void do_writer_sends();
    void handle_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds);

    Telem_Client *telem_client = NULL;
    MAVLink_Reader *reader;
};

class Log_Download_Program : public Common_Tool {
public:
    Log_Download_Program() :
        Common_Tool()
        { }

    void run();

    void parse_arguments(int argc, char *argv[]);
    const char *program_name();

    void pack_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds) override;
    void handle_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds) override;
    
private:
    void usage();
    void sighup_received_tophalf() override;
    void do_idle_callbacks() override;
    uint32_t select_timeout_us() override;

    void do_writer_sends();

    MAVLink_Writer *_writer;

    LD_Client *client;

    long _argc = 0;
    char **_argv = NULL;

    bool debug_mode = false;
    bool serial_port = false;
    bool list = false;

    char *serial_port_path = nullptr;
    uint32_t serial_port_baud = 57600;
    bool serial_port_flow = true;

    std::vector<uint16_t> logs_to_download;

    void create_telem_forwarder_client();
    void create_serial_client();

};
