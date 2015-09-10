#include "mh_imagetagger.h"

// FIXME: should be able to factor this stuff?

void MH_ImageTagger::handle_decoded_message(uint64_t T, mavlink_gps_raw_int_t &msg) {
    // ::fprintf(stderr, "gps timestamp: %ld\n", msg.time_usec);
    // ::fprintf(stderr, "message timestamp: %lu\n", T);
    // ::fprintf(stderr, "offset: %ld\n", _time_offset);
    std::vector<ImageTagger::Image_Info*>::iterator next;
    for (std::vector<ImageTagger::Image_Info*>::iterator it = _start_image;
         it != _images.end();) {
        uint64_t timestamp_image = (*it)->timestamp;
        timestamp_image += _time_offset;
        // ::fprintf(stderr, "   image timestamp=%lu\n", timestamp_image);
        if ((*it)->have_gps_raw_int) {
            continue;
        }
        if (T > timestamp_image) {
            // ::fprintf(stderr, "      Match for image (%s)\n", (*it)->path.c_str());
            // it = _images.erase(it);
            (*it)->latitude = msg.lat / 10000000.0f;
            (*it)->longitude = msg.lon / 10000000.0f;
            (*it)->altitude = msg.alt/100.0f;
            (*it)->have_gps_raw_int = true;

            if ((*it)->have_all_data()) {
                ++it;
                _start_image = it;
            } else {
                ++it;
            }
        } else {
            ++it;
        }
    }

    memcpy(&previous_gps_raw_int_message, &msg, sizeof(msg));
}


void MH_ImageTagger::handle_decoded_message(uint64_t T, mavlink_mount_status_t &msg) {
    // ::fprintf(stderr, "gps timestamp: %ld\n", msg.time_usec);
    // ::fprintf(stderr, "message timestamp: %lu\n", T);
    // ::fprintf(stderr, "offset: %ld\n", _time_offset);
    std::vector<ImageTagger::Image_Info*>::iterator next;
    for (std::vector<ImageTagger::Image_Info*>::iterator it = _start_image;
         it != _images.end();) {
        uint64_t timestamp_image = (*it)->timestamp;
        timestamp_image += _time_offset;
        // ::fprintf(stderr, "   image timestamp=%lu\n", timestamp_image);
        if ((*it)->have_mount_status) {
            continue;
        }
        if (T > timestamp_image) {
            // ::fprintf(stderr, "      ms Match for image (%s)\n", (*it)->path.c_str());
            (*it)->gimbal_pitch = msg.pointing_a / 100.0f;
            (*it)->gimbal_roll = msg.pointing_b / 100.0f;
            (*it)->gimbal_yaw = msg.pointing_c / 100.0f;
            (*it)->have_mount_status = true;

            if ((*it)->have_all_data()) {
                ++it;
                _start_image = it;
            } else {
                ++it;
            }
        } else {
            ++it;
        }
    }

    memcpy(&previous_mavlink_mount_status_message, &msg, sizeof(msg));
}
