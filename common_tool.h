#include "util.h"
#include "analyzer_util.h"
#include "INIReader.h"

class Common_Tool {
public:
    Common_Tool() :
        _config(NULL),
        config_filename(default_config_filename),
        _sighup_received(false)
        {
        }

    void sighup_handler(int signal);

protected:
    class INIReader *_config;
    class INIReader *get_config();
    const char *default_config_filename = "/etc/sololink.conf";
    const char * config_filename;

    virtual void sighup_received_tophalf();

    bool _sighup_received; // FIXME: scope

    void select_loop();

    virtual void pack_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds);
    virtual void handle_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds);
    virtual void do_idle_callbacks();
    
private:
    void check_fds_are_empty_after_select(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t nfds);

};
