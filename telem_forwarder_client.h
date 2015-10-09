#include "INIReader.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "telem_client.h"

class Telem_Forwarder_Client : public Telem_Client {
public:
    Telem_Forwarder_Client(uint8_t *buf, uint32_t buflen) :
        _buflen_content(0),
        fd_telem_forwarder(-1),
        _buf(buf),
        _buflen(buflen)
        { }

    uint32_t handle_recv();
    bool telem_forwarder_packet(uint8_t *pkt, uint16_t bpktlen);

    void configure(INIReader *config);
    void pack_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds);
    void handle_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds);

    // FIXME: scope
    uint32_t _buflen_content;

    int32_t do_send(const char *buf, const uint32_t buflen);

private:
    int fd_telem_forwarder;

    struct sockaddr_in sa; // our send-from address
    struct sockaddr_in sa_tf; /* telem_forwarder's address */

    void telem_forwarder_loop();
    void create_and_bind();
    void pack_telem_forwarder_sockaddr(INIReader *config);
    bool sane_telem_forwarder_packet(uint8_t *pkt, uint16_t bpktlen);
    uint8_t *_buf; // receive buffer
    uint32_t _buflen; // receive buffer len

};
