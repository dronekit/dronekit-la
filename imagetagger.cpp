#include "imagetagger.h"

#include "mh_imagetagger.h"

#include "mavlink_reader.h"

#include <syslog.h>
#include "la-log.h"

#include <sys/stat.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>

#include <algorithm>

void ImageTagger::parse_filepath(const char *filepath)
{
    int fd = open(filepath, O_RDONLY);
    if (fd == -1) {
        fprintf(stderr, "Failed to open (%s): %s\n", filepath, strerror(errno));
        exit(1);
    }


    return reader->parse_fd(fd);
}

uint64_t find_image_timesstamp(const std::string path)
{
    char tmp_template[] = "/tmp/imageinfo.XXXXXX";
    char *exif_output_filepath = mktemp(tmp_template);
    std::string command = string_format("exiftool  -d '%%s' -s3 -CreateDate %s >%s", path.c_str(), exif_output_filepath);
    int system_ret = system(command.c_str());
    if (system_ret == -1) {
        ::fprintf(stderr, "Unable to find timestamp (%s): %s\n", command.c_str(), strerror(errno));
        return 0; //?
    }
    if (system_ret != 0) {
        ::fprintf(stderr, "Warn: Non-zero system_return code for (%s)\n", command.c_str());
        return 0; //?
    }
    char buf[128];
    int fd = open(exif_output_filepath, O_RDONLY);
    if (fd == -1) {
        ::fprintf(stderr, "Failed to open (%s): %s\n", exif_output_filepath, strerror(errno));
        return 0; //?
    }
    if (read(fd, buf, 128) <= 0) {
        ::fprintf(stderr, "Warn: No output for (%s)\n", command.c_str());
    }
    
    unlink(exif_output_filepath);
    uint64_t ret = strtoul(buf,NULL,10);
    // ::fprintf(stderr, "Returning (%lu) for (%s)\n", ret, path.c_str());
    return ret *1000000;
}

void ImageTagger::do_tagging(Image_Info *info)
{
    char tmp_template[] = "/tmp/imageinfo.XXXXXX";
    char *exif_output_filepath = mktemp(tmp_template);
    
    std::string command = string_format(
        "exiftool " \
        "-config ExifTool_config " \
        "%s " \
        "-GPSLatitude=%f "    \
        "-GPSLatitudeRef=%d " \
        "-GPSLongitude=%f " \
        "-GPSLongitudeRef=%d " \
        "-GPSAltitude=%f " \
        "-GPSDestBearing=%f " \
        "-GPSDestBearingRef=%s " \
        "-GPSRoll=%f " \
        "-GPSPitch=%f " \
        ">%s",
        info->path.c_str(),
        fabs(info->latitude),
        int(info->latitude / fabs(info->latitude)),
        fabs(info->longitude),
        int(info->longitude / fabs(info->longitude)),
        info->altitude,
        (info->gimbal_yaw < 0) ? (360+info->gimbal_yaw) : info->gimbal_yaw,
        "M",
        info->gimbal_roll,
        info->gimbal_pitch,
        exif_output_filepath);
    ::fprintf(stderr, "Info: Executing (%s)\n", command.c_str());
    int system_ret = system(command.c_str());
    if (system_ret == -1) {
        ::fprintf(stderr, "Unable to tag (%s): %s\n", command.c_str(), strerror(errno));
        return; //?
    }
    if (system_ret != 0) {
        ::fprintf(stderr, "Warn: Non-zero system_return code for (%s)\n", command.c_str());
        return; //?
    }
    // char buf[128];
    // int fd = open(exif_output_filepath, O_RDONLY);
    // if (fd == -1) {
    //     ::fprintf(stderr, "Failed to open (%s): %s\n", exif_output_filepath, strerror(errno));
    //     return; //?
    // }
    // if (read(fd, buf, 128) <= 0) {
    //     ::fprintf(stderr, "Warn: No output for (%s)\n", command.c_str());
    // }
    
    unlink(exif_output_filepath);
    return;
}

std::vector<class ImageTagger::Image_Info*> ImageTagger::get_image_info(const char *dirpath)
{
    std::vector<ImageTagger::Image_Info*> ret;
    
    DIR *dh = opendir(dirpath);
    if (dh == NULL) {
        fprintf(stderr, "Failed to open (%s): %s\n", dirpath, strerror(errno));
        exit(1);
    }
    struct dirent *ent;
    for(ent = readdir(dh); ent != NULL; ent = readdir(dh)) {
        if (streq(ent->d_name, ".") || streq(ent->d_name, "..")) {
            continue;
        }
        if (!strstr(ent->d_name, ".JPG")) {
            ::fprintf(stderr, "Warning: Skipping non-imagey-looking thing (%s)\n", ent->d_name);
            continue;
        }
        ::fprintf(stderr, "Image: %s\n", ent->d_name);
        ImageTagger::Image_Info *entry = new ImageTagger::Image_Info();
        if (entry == NULL) {
            fprintf(stderr,"Failed to create image info: %s\n", strerror(errno));
            abort();
        }
        entry->path = std::string(dirpath) + "/" + std::string(ent->d_name); // FIXME
        entry->timestamp = find_image_timesstamp(entry->path);
        ret.push_back(entry);
    }

    struct lesser {
        bool operator()( const ImageTagger::Image_Info *lx, const ImageTagger::Image_Info *rx ) const {
            // return strcmp(lx->path.c_str(), rx->path.c_str()) > 0;
            return lx->timestamp < rx->timestamp;
        }
    };

    std::sort(ret.begin(), ret.end(), lesser());
    return ret;
}

void ImageTagger::run()
{
    la_log(LOG_INFO, "ImageTagger starting: built " __DATE__ " " __TIME__);
    // signal(SIGHUP, sighup_handler);

    INIReader *config = get_config();

    reader = new MAVLink_Reader(config);
    reader->set_is_tlog(true);

    if (_imagedir == NULL) {
        usage();
        exit(1);
    }

    if (_pathname == NULL) {
        usage();
        exit(1);
    }

    if (_time_offset_string != NULL) {
        if (strstr(_time_offset_string, ":")) {
            fprintf(stderr, "warn: time offset has minutes - sorry, not handled");
        } else {
            // hours to microseconds:
            _time_offset = strtol(_time_offset_string,NULL,10) * 60*60*1000000;
        }
    }
    std::vector<ImageTagger::Image_Info*> images = get_image_info(_imagedir);
    // for (std::vector<ImageTagger::Image_Info*>::iterator it = images.begin();
    //      it != images.end(); ++it) {
    //     ::fprintf(stderr,"%s\n", (*it)->path.c_str());
    // }

    MH_ImageTagger *tagger = new MH_ImageTagger(-1, NULL, images, _time_offset);
    if (tagger != NULL) {
        reader->add_message_handler(tagger, "Tagger");
    } else {
        la_log(LOG_ERR, "Failed to create tagger");
        abort();
    }

    parse_filepath(_pathname);

    for (std::vector<ImageTagger::Image_Info*>::iterator it = images.begin();
         it != images.end(); ++it) {
        ::fprintf(stderr,"%s\n", (*it)->path.c_str());
        ::fprintf(stderr,"     latitude:%f\n", (*it)->latitude);
        ::fprintf(stderr,"    longitude:%f\n", (*it)->longitude);
        ::fprintf(stderr,"     altitude:%f\n", (*it)->altitude);
        ::fprintf(stderr,"         roll:%f\n", (*it)->gimbal_roll);
        ::fprintf(stderr,"        pitch:%f\n", (*it)->gimbal_pitch);
        ::fprintf(stderr,"          yaw:%f\n", (*it)->gimbal_yaw);
        do_tagging((*it));
    }
}

void ImageTagger::usage()
{
    ::printf("Usage:\n");
    ::printf("%s [OPTION] TLOG IMAGEDIR\n", program_name());
    ::printf(" -c filepath      use config file filepath\n");
    ::printf(" -h               display usage information\n");
    ::printf(" -z               time offset for images (HH:MM)\n");
    ::printf("\n");
    ::printf("Example: %s 1.solo.tlog my-image-dir\n", program_name());
    exit(0);
}
const char *ImageTagger::program_name()
{
    if (_argv == NULL) {
        return "[Unknown]";
    }
    return _argv[0];
}


void ImageTagger::parse_arguments(int argc, char *argv[])
{
    int opt;
    _argc = argc;
    _argv = argv;

    while ((opt = getopt(argc, argv, "hc:ts:z:")) != -1) {
        switch(opt) {
        case 'h':
            usage();
            break;
        case 'c':
            config_filename = optarg;
            break;
        case 'z':
            _time_offset_string = optarg;
            break;
        }
    }
    if (optind < argc) {
        _pathname = argv[optind++];
    }
    if (optind < argc) {
        _imagedir = argv[optind++];
    }
}

/*
* main - entry point
*/
int main(int argc, char* argv[])
{
    ImageTagger program;
    program.parse_arguments(argc, argv);
    program.run();
}
