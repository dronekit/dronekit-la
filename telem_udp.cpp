#include "telem_udp.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include "la-log.h"

#ifdef _WIN32
#include <ws2tcpip.h>
#endif

#define UNUSED __attribute__ ((unused))

void Telem_UDP::pack_select_fds(fd_set &fds_read,
                                       fd_set &fds_write UNUSED,
                                       fd_set &fds_err,
                                       uint8_t &nfds)
{
    FD_SET(fd, &fds_read);
    FD_SET(fd, &fds_err);

    if (fd >= nfds) {
        nfds = fd + 1;
    }
}


void Telem_UDP::handle_select_fds(fd_set &fds_read,
                                         fd_set &fds_write UNUSED,
                                         fd_set &fds_err,
                                         uint8_t &nfds UNUSED)
{
    /* check for packets from source */
    if (FD_ISSET(fd, &fds_err)) {
        FD_CLR(fd, &fds_err);
        la_log(LOG_ERR, "select(fd): %s", strerror(errno));
    }

    if (FD_ISSET(fd, &fds_read)) {
        FD_CLR(fd, &fds_read);
        _recv_buflen_content = handle_recv();
        // ::fprintf(stderr, "received %u bytes\n", _recv_buflen_content);
    }
}


void Telem_UDP::create_and_bind()
{
    int _fd;
    struct sockaddr_in sa;

    _fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (_fd < 0) {
        perror("socket");
        abort();
    }

    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_addr.s_addr = htonl(INADDR_ANY);
    sa.sin_port = 0; // we don't care what our port is

    if (bind(_fd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
        perror("bind");
        abort();
    }

    fd = _fd;
}


std::string Telem_UDP::udp_ip(INIReader *config) const
{
    return config->Get("dflogger", "udp_ip", "127.0.0.1");
}

uint16_t Telem_UDP::udp_port(INIReader *config) const
{
    return config->GetInteger("dflogger", "udp_port", 14550);
}

void Telem_UDP::pack_sockaddr(INIReader *config)
{
    // uint16_t tf_port = config->GetInteger("solo", "telem_forward_port", 14560);
    const uint16_t tf_port = udp_port(config);
    const std::string ip = udp_ip(config);

    la_log(LOG_INFO, "df-udp: connecting to %s:%u", ip.c_str(), tf_port);
    memset(&sa_tf, 0, sizeof(sa_tf));
    sa_tf.sin_family = AF_INET;
#ifdef _WIN32
    sa_tf.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
#else
    inet_aton(ip.c_str(), &sa_tf.sin_addr); // useful for debugging
#endif
    sa_tf.sin_port = htons(tf_port);
}

bool Telem_UDP::sane_recv_buf(uint8_t *pkt UNUSED, uint16_t pktlen UNUSED) const
{
    return true;
}

uint32_t Telem_UDP::handle_recv()
{
    // ::printf("Receiving packet into position %u\n", _buflen_content);
    socklen_t sa_len = sizeof(sa);
    uint16_t res = recvfrom(fd,
                            (char*)&_recv_buf[_recv_buflen_content],
                            _recv_buflen-_recv_buflen_content,
                            0,
                            (struct sockaddr*)&sa,
                            &sa_len);

    if (!sane_recv_buf(_recv_buf, res)) {
	return 0;
    }

    return res;
}

bool Telem_UDP::send_message(const mavlink_message_t &msg)
{
    if (send_buffer_space_free() < 1) {
        // dropped_packets++;
        return false;
    }
    memcpy(&_send_buf[_send_buf_stop++], (char*)&msg, sizeof(msg));
    if (_send_buf_stop >= send_buf_size()) {
        _send_buf_stop = 0;
    }
    return true;
}


void Telem_UDP::do_writer_sends()
{
    char buf[1024]; // large enough...

    while (_send_buf_start != _send_buf_stop) {
        mavlink_message_t &msg = _send_buf[_send_buf_start];
        uint16_t messageLen = mavlink_msg_to_send_buffer((uint8_t*)buf,&msg);

        int32_t bytes_sent =
            sendto(fd, buf, messageLen, 0,
                   (struct sockaddr *)&sa_tf, sizeof(struct sockaddr));
        if (bytes_sent == -1) {
            la_log(LOG_INFO, "Failed sendto: %s", strerror(errno));
            // we drop the message anyway!
        }
        _send_buf_start++;
        if (_send_buf_start >= send_buf_size()) {
            _send_buf_start = 0;
        }
    }

    return;
}

void Telem_UDP::configure(INIReader *config)
{
    pack_sockaddr(config);

}

void Telem_UDP::init() {
    create_and_bind();
}
