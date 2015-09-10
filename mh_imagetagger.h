#ifndef MH_IMAGETAGGER_H
#define MH_IMAGETAGGER_H

#include "mavlink_message_handler.h"
#include "mh_imagetagger.h"
#include "imagetagger.h"

#include <vector>

class MH_ImageTagger : public MAVLink_Message_Handler {

public:
    MH_ImageTagger(int fd, struct sockaddr_in *sa, std::vector<ImageTagger::Image_Info*> images, int64_t time_offset) :
	MAVLink_Message_Handler(fd, sa),
        _images(images),
        _time_offset(time_offset),
        _start_image(_images.begin())
        {
            start_time = now();
        }

    // void end_of_log(uint32_t packet_count);

private:
    std::vector<ImageTagger::Image_Info*> _images;
    int64_t _time_offset;

    void handle_decoded_message(uint64_t T, mavlink_gps_raw_int_t &msg);
    void handle_decoded_message(uint64_t T, mavlink_mount_status_t &msg);

    uint64_t start_time;
    mavlink_gps_raw_int_t previous_gps_raw_int_message;
    mavlink_gps_raw_int_t previous_mavlink_mount_status_message;
    std::vector<ImageTagger::Image_Info*>::iterator _start_image;
};

#endif
    
