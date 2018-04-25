#include <signal.h>
#include <stdio.h> // for snprintf
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>

#include "la-log.h"
#include "util.h"
#include "mavlink_c_library/ardupilotmega/mavlink.h"

#include "log_download.h"

bool Log_Download::configure(INIReader *config)
{
    if (!MAVLink_Message_Handler::configure(config)) {
        return false;
    }
    return true;
}

void Log_Download::handle_message(uint64_t timestamp, mavlink_message_t &msg)
{
    most_recent_sender_system_id = msg.sysid;
    most_recent_sender_component_id = msg.compid;
    MAVLink_Message_Handler::handle_message(timestamp, msg);
}

void Log_Download::handle_decoded_message(uint64_t T UNUSED, mavlink_log_entry_t &msg)
{
    if (msg.id != id_to_fetch) {
        return;
    }

    log_info = new LogEntry();
    log_info->id = msg.id;
    log_info->size = msg.size;
    log_info->time_utc = msg.time_utc;
}

void Log_Download::write_data(mavlink_log_data_t &msg)
{
    if (output_fh == -1) {
        ::fprintf(stderr, "opening output_fh\n");
        char filename[120];
        snprintf(filename, sizeof(filename), "log%u.bin", id_to_fetch);
        output_fh = open(filename, O_WRONLY|O_TRUNC|O_CREAT, 0644);
        if (output_fh == -1) {
            ::fprintf(stderr, "Failed to open (%s): %s", filename, strerror(errno));
            abort();
        }
        // expand the file:
        if (lseek(output_fh, log_info->size, SEEK_SET) == -1) {
            ::fprintf(stderr, "Failed to seek\n");
            abort();
        }
    }
    last_data_receieved = clock_gettime_us(CLOCK_MONOTONIC);
    if (lseek(output_fh, msg.ofs, SEEK_SET) == -1) {
        ::fprintf(stderr, "Failed to seek\n");
        abort();
    }
    // ::fprintf(stderr, "Writing %u bytes at offset %u\n", msg.count, msg.ofs);
    if (write(output_fh, msg.data, msg.count) != msg.count) {
        ::fprintf(stderr, "Short write\n");
        abort();
    }
}

void Log_Download::handle_decoded_message(uint64_t T UNUSED, mavlink_log_data_t &msg)
{
    if (msg.id != id_to_fetch) {
        return;
    }
    // la_log(LOG_ERR, "data: ofs=%u count=%u", msg.ofs, msg.count);
//    ::fprintf(stderr, "data: ofs=%u count=%u\n", msg.ofs, msg.count);

    for (auto it=gaps.begin(); it != gaps.end(); it++) {
        Gap *gap = *it;
        if (msg.ofs >= gap->start &&
            msg.ofs < gap->start+gap->len) {
            write_data(msg);
        }
        // ::fprintf(stderr, "msg.ofs=%u gap->start=%u\n", msg.ofs, gap->start);
        if (msg.ofs == gap->start) {
            // msg is at start of gap
            if (msg.count >= gap->len) {
                // this gap is all done!
                gap->len = 0;
            } else {
                gap->start += msg.count;
                if (msg.count > gap->len) {
                    gap->len = 0;
                } else {
                    gap->len -= msg.count;
                }
            }
        } else if (msg.ofs + msg.count >= gap->start+gap->len) {
            // msg is at end of gap
            if (msg.ofs + msg.count == gap->start+gap->len) {
                // right at end of buffer
                gap->len -= (gap->start+gap->len)-msg.ofs;
            }
        } else {
            ::fprintf(stderr, "Split required\n");
            abort();
        }
    }
}


void Log_Download::send_log_request_list(uint16_t id)
{
    la_log(LOG_INFO, "mh-ld: sending log_request_list (%u) to %u/%u", id, target_system_id, target_component_id);

    mavlink_message_t msg;
    mavlink_msg_log_request_list_pack(system_id,
                                      component_id,
                                      &msg,
                                      target_system_id,
                                      target_component_id,
                                      id,
                                      id);

    _mavlink_writer->send_message(msg);
}

uint64_t Log_Download::time_in_state() const
{
    return (clock_gettime_us(CLOCK_MONOTONIC) - state_start_time);
}

void Log_Download::send_data_request(Gap *gap)
{
    mavlink_message_t msg;
    mavlink_msg_log_request_data_pack(system_id,
                                      component_id,
                                      &msg,
                                      target_system_id,
                                      target_component_id,
                                      id_to_fetch,
                                      gap->start,
                                      gap->len);

    _mavlink_writer->send_message(msg);
}

void Log_Download::sort_gaps()
{
    if (gaps.front()->len == 0) {
        gaps.erase(gaps.begin());
    }
}


void Log_Download::run_statemachine()
{
    switch(state) {
    case WAITING_FOR_SYSTEM:
        if (target_system_id == 0) {
            if (most_recent_sender_component_id == 0) {
                return;
            }
        }
        target_system_id = most_recent_sender_system_id;
        target_component_id = most_recent_sender_component_id;
        state = NEXT_LOG;
        // fall-through
    case NEXT_LOG: {
        if (logs_to_download.size() == 0) {
            state = DONE;
            return;
        }
        id_to_fetch = logs_to_download.back();
        logs_to_download.pop_back();
        ::fprintf(stderr, "Downloading log %u\n", id_to_fetch);
        delete log_info;
        log_info = nullptr;
        state = SEND_GET_LOG_INFO;
    }
        // fall-through
    case SEND_GET_LOG_INFO:
        send_log_request_list(id_to_fetch);
        state = SENT_GET_LOG_INFO;
        return;
    case SENT_GET_LOG_INFO: {
        if (log_info == nullptr) {
            if (time_in_state() > 1000000) {
                state = SEND_GET_LOG_INFO;
            }
            return;
        }
        Gap *gap = new Gap();
        gap->start = 0;
        gap->len = log_info->size;
        gaps.push_back(gap);
        state = FILL_GAPS;
        download_start_time = clock_gettime_us(CLOCK_MONOTONIC);
    }
        // fall-through
    case FILL_GAPS: {
        bool new_gap = false;
        if (gaps.front()->len == 0) {
            sort_gaps();
            new_gap = true;
        }
        if (gaps.size() == 0) {
            state = DONE;
            close(output_fh);
            output_fh = -1;
            return;
        }
        if (new_gap ||
            clock_gettime_us(CLOCK_MONOTONIC) - last_data_receieved > 100000) {
            send_data_request(gaps.front());
        }
        return;
    }
    case DONE: {
        const uint64_t delta_time = clock_gettime_us(CLOCK_MONOTONIC) - download_start_time;
        const float rate = (log_info->size/1024.0)/(delta_time/1000000.0);
        ::fprintf(stderr, "Download done (%u bytes) (%lus) (%fkB/s)\n",
                  log_info->size,
                  delta_time/1000000,
                  rate);
        exit(0);
    }
    }
}

void Log_Download::idle_10Hz()
{
    run_statemachine();
    if (state != last_state) {
        last_state = state;
        state_start_time = clock_gettime_us(CLOCK_MONOTONIC);
    }
}
