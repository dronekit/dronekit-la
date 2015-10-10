#ifndef _TELEM_CLIENT_H
#define _TELEM_CLIENT_H

class Telem_Client {
public:
    virtual int32_t write(const char *buf, uint32_t buflen) = 0;
    virtual void pack_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds) = 0;
    virtual void configure(INIReader *config) = 0;
    virtual void handle_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds) = 0;

    virtual void init() = 0;

    // FIXME: scope
    uint32_t _buflen_content = 0;

private:
         
};

#endif
