#include "INIReader.h"

#ifdef _WIN32
#include <Winsock2.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

#include "telem_client.h"

class Telem_UDP : public Telem_Client {
public:

    Telem_UDP() :
        Telem_Client()
        { }

    uint32_t handle_recv();
    void init() override;

    void configure(INIReader *config) override;
    void pack_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds) override;
    void handle_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds) override;

    void do_writer_sends() override;
    bool send_message(const mavlink_message_t &msg) override;
    bool any_data_to_send() override {
        return _send_buf_start != _send_buf_stop;
    }

private:
    int fd = -1;

    struct sockaddr_in sa = {}; // our send-from address
    struct sockaddr_in sa_destination = {}; /* udp address to connect to */

    void create_and_bind(INIReader *config);
    void pack_sockaddr(INIReader *config);
    bool sane_packet(uint8_t *pkt, uint16_t bpktlen);
    virtual bool sane_recv_buf(uint8_t *pkt, uint16_t pktlen) const;

    /* send buffer stuff: */
    mavlink_message_t _send_buf[256]; // way too big?
    uint32_t send_buf_size() const override {
        return 256;
    }

    std::string udp_ip(INIReader *config) const;
    uint16_t udp_port(INIReader *config) const;
    bool is_server(INIReader *config) const;

    bool _is_server = false;
};
