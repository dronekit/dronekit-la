#include "telem_forwarder_client.h"

#include <string.h>

#include "la-log.h"

/*
* create_and_bind - create a socket and bind it to a local UDP port
*
* Used to create the socket on the upstream side that receives from and sends
* to telem_forwarder
*
* Returns fd on success, -1 on error.
*/
void Telem_Forwarder_Client::pack_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds)
{
    FD_SET(fd_telem_forwarder, &fds_read);
    FD_SET(fd_telem_forwarder, &fds_err);

    if (fd_telem_forwarder >= nfds) {
        nfds = fd_telem_forwarder + 1;
    }
}

        
void Telem_Forwarder_Client::handle_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds)
{
    /* check for packets from telem_forwarder */
    if (FD_ISSET(fd_telem_forwarder, &fds_err)) {
        FD_CLR(fd_telem_forwarder, &fds_err);
        unsigned skipped = 0;
        // if ((skipped = can_log_error()) >= 0)
        la_log(LOG_ERR, "[%u] select(fd_telem_forwarder): %s", skipped, strerror(errno));
    }

    if (FD_ISSET(fd_telem_forwarder, &fds_read)) {
        FD_CLR(fd_telem_forwarder, &fds_read);
        _buflen_content = handle_recv();
        ::fprintf(stderr, "received %u bytes\n", _buflen_content);
    }
}


void Telem_Forwarder_Client::create_and_bind()
{
    int fd;
    struct sockaddr_in sa;

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("socket");
        abort();
    }

    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_addr.s_addr = htonl(INADDR_ANY);
    sa.sin_port = 0; // we don't care what our port is

    if (bind(fd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
        perror("bind");
        abort();
    }

    fd_telem_forwarder = fd;
} /* create_and_bind */


void Telem_Forwarder_Client::pack_telem_forwarder_sockaddr(INIReader *config)
{
    uint16_t tf_port = config->GetInteger("solo", "telem_forward_port", 14560);
    std::string ip = config->Get("solo", "soloIp", "10.1.1.10");

    la_log(LOG_INFO, "df-tfc: connecting to telem-forwarder at %s:%u", ip.c_str(), tf_port);
    memset(&sa_tf, 0, sizeof(sa_tf));
    sa_tf.sin_family = AF_INET;
    //    sa_tf.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    inet_aton(ip.c_str(), &sa_tf.sin_addr); // useful for debugging
    sa_tf.sin_port = htons(tf_port);
}

bool can_log_error() {
    return true;
}
bool Telem_Forwarder_Client::sane_telem_forwarder_packet(uint8_t *pkt, uint16_t pktlen)
{
    if (sa.sin_addr.s_addr != sa_tf.sin_addr.s_addr) {
	unsigned skipped = 0;
	// if ((skipped = can_log_error()) >= 0)
	    la_log(LOG_ERR, "[%u] received packet not from solo (0x%08x)",
		   skipped, sa.sin_addr.s_addr);
	return false;
    }
    if (pktlen < 8) { 
	unsigned skipped;
	if ((skipped = can_log_error()) >= 0)
	    la_log(LOG_ERR, "[%u] received runt packet (%d bytes)",
		   skipped, pktlen);
	return false;
    }
    if (pkt[0] != 254) {
	unsigned skipped;
	if ((skipped = can_log_error()) >= 0)
	    la_log(LOG_ERR, "[%u] received bad magic (0x%02x)",
		   skipped, pkt[0]);
	return false;
    }
    if (pkt[1] != (pktlen - 8)) {
	unsigned skipped;
	if ((skipped = can_log_error()) >= 0)
	    la_log(LOG_ERR, "[%u] inconsistent length (%u, %u)",
		   skipped, pkt[1], pktlen);
	return false;
    }
    return true;
}

uint32_t Telem_Forwarder_Client::handle_recv()
{
    // ::printf("Receiving packet\n");
    /* packet from telem_forwarder */
    socklen_t sa_len = sizeof(sa);
    uint16_t res = recvfrom(fd_telem_forwarder, &_buf[_buflen_content], _buflen-_buflen_content, 0, (struct sockaddr*)&sa, &sa_len);

    /* We get one mavlink packet per udp datagram. Sanity checks here
       are: must be from solo's IP and have a valid mavlink header. */
    // FIXME: we don't necessarily get just one packet/buffer!
    if (!sane_telem_forwarder_packet(_buf, res)) {
	return 0;
    }

    return res;
}


void Telem_Forwarder_Client::configure(INIReader *config)
{
    /* prepare sockaddr used to contact telem_forwarder */
    pack_telem_forwarder_sockaddr(config);

    /* Prepare a port to receive and send data to/from telem_forwarder */
    /* does not return on failure */
    create_and_bind();
}
