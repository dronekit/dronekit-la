#include "INIReader.h"

#include "telem_client.h"

class Telem_Serial : public Telem_Client {
public:
    Telem_Serial(uint8_t *buf, uint32_t buflen) :
        _buf(buf),
        _buflen(buflen)
        { }

    uint32_t handle_read();

    void configure(INIReader *config);
    void init() override;

    void pack_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds);
    void handle_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds);

    int32_t write(const char *buf, const uint32_t buflen);

    void open_serial_port();

private:
    int fd;

    uint8_t *_buf; // receive buffer
    uint32_t _buflen; // receive buffer len

    std::string serialPortName;
    uint32_t serialBaud;
    bool serialFlow;
};
