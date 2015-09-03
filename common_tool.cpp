#include "common_tool.h"

#include <errno.h>
#include <sys/stat.h>
#include <unistd.h>

#include "la-log.h"

INIReader *Common_Tool::get_config()
{
    if (config_filename == NULL) {
        abort();
    }
    struct stat buf;
    if (stat(config_filename, &buf) == -1) {
        if (errno == ENOENT) {
            if (! streq(default_config_filename, config_filename)) {
                la_log(LOG_CRIT, "Config file (%s) does not exist", config_filename);
                exit(1);
            }
        } else {
            la_log(LOG_CRIT, "Failed to stat (%s): %s\n", config_filename, strerror(errno));
            exit(1);
        }
        _config = new INIReader(config_filename);
        if (_config == NULL) {
            la_log(LOG_CRIT, "Failed to create config from (%s)\n", config_filename);
            exit(1);
        }
    }
    if (_config == NULL) {
        _config = new INIReader("/dev/null");
    }
    return _config;
}



void Common_Tool::check_fds_are_empty_after_select(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t nfds)
{
    for (uint8_t i=0; i<nfds; i++) {
        if (FD_ISSET(i, &fds_read)) {
            la_log(LOG_ERR, "fds_read not empty");
            break;
        }
        if (FD_ISSET(i, &fds_write)) {
            la_log(LOG_ERR, "fds_write not empty");
            break;
        }
        if (FD_ISSET(i, &fds_err)) {
            la_log(LOG_ERR, "fds_err not empty");
            break;
        }
    }
}

void Common_Tool::pack_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds)
{
}

void Common_Tool::handle_select_fds(fd_set &fds_read, fd_set &fds_write, fd_set &fds_err, uint8_t &nfds)
{
}

void Common_Tool::sighup_received_tophalf()
{
}
void Common_Tool::sighup_handler(int signal)
{
    _sighup_received = true;
}
void Common_Tool::do_idle_callbacks()
{
}
void Common_Tool::select_loop()
{
    while (1) {
	if (_sighup_received) {
            sighup_received_tophalf();
	    _sighup_received = false;
	}
        /* Wait for a packet, or time out if no packets arrive so we always
           periodically log status and check for new destinations. Downlink
           packets are on the order of 100/sec, so the timeout is such that
           we don't expect timeouts unless solo stops sending packets. We
           almost always get a packet with a 200 msec timeout, but not with
           a 100 msec timeout. (Timeouts don't really matter though.) */

	struct timeval timeout;

        fd_set fds_read;
        fd_set fds_write;
        fd_set fds_err;
        uint8_t nfds = 0;
        FD_ZERO(&fds_read);
        FD_ZERO(&fds_write);
        FD_ZERO(&fds_err);
        pack_select_fds(fds_read, fds_write, fds_err, nfds);

        timeout.tv_sec = 0;
        timeout.tv_usec = 200000;
        int res = select(nfds, &fds_read, &fds_write, &fds_err, &timeout);

        if (res < 0) {
            unsigned skipped = 0;
            // if ((skipped = can_log_error()) >= 0)
            la_log(LOG_ERR, "[%u] select: %s", skipped, strerror(errno));
            /* this sleep is to avoid soaking the CPU if select starts
               returning immediately for some reason */
	    /* previous code was not checking errfds; we are now, so
	       perhaps this usleep can go away -pb20150730 */
            usleep(10000);
            continue;
        }

        if (res == 0) {
	  // select timeout
        }

        handle_select_fds(fds_read, fds_write, fds_err, nfds);

        check_fds_are_empty_after_select(fds_read, fds_write, fds_err, nfds);

	do_idle_callbacks();
    } /* while (1) */
}
