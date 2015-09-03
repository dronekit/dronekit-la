#include "INIReader.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

class Telem_Forwarder_Client {
public:
    Telem_Forwarder_Client(uint8_t *buf, uint32_t buflen) :
        fd_telem_forwarder(-1),
        _buflen_content(0),
        _buf(buf),
        _buflen(buflen)
        { }

    // FIXME: scope
    int fd_telem_forwarder;
    uint32_t handle_recv();
    bool telem_forwarder_packet(uint8_t *pkt, uint16_t bpktlen);
    struct sockaddr_in sa_tf; /* solo's address */

    void configure(INIReader *config);
    void pack_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds);
    void handle_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds);

    // FIXME: scope
    uint32_t _buflen_content;

private:
    struct sockaddr_in sa;
    void telem_forwarder_loop();
    void create_and_bind();
    void pack_telem_forwarder_sockaddr(INIReader *config);
    bool sane_telem_forwarder_packet(uint8_t *pkt, uint16_t bpktlen);
    uint8_t *_buf;
    uint32_t _buflen;
};
