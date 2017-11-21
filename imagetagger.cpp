#include "imagetagger.h"

#include "mh_imagetagger.h"

#include "mavlink_reader.h"

#include "dataflash_reader.h"
#include "dataflash_textdump_reader.h"

#include <syslog.h>
#include "la-log.h"

#include <sys/stat.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>

#include <algorithm>

// copied in from loganalyzer.cpp; maybe use CommonTool or something?
// http://stackoverflow.com/questions/874134/find-if-string-ends-with-another-string-in-c
inline bool ends_with(std::string const & value, std::string const & ending)
{
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

void ImageTagger::parse_path(const char *path)
{
    bool do_stdin = false;

    log_format_t log_format = log_format_none;
    if (!strcmp(path, "-")) {
        do_stdin = true;
    } else if (ends_with(path, ".BIN") ||
               ends_with(path, ".bin")) {
        log_format = log_format_df;
    } else if (ends_with(path, ".log") ||
               ends_with(path, ".LOG")) {
        log_format = log_format_log;
    } else if (ends_with(path, ".tlog") ||
               ends_with(path, ".TLOG")) {
        log_format = log_format_tlog;
    }

    if (force_format() != log_format_none) {
        log_format = force_format();
    }

    if (log_format == log_format_none) {
        if (do_stdin) {
            ::fprintf(stderr, "You asked to parse stdin but did not force a format type\n");
        } else {
            ::fprintf(stderr, "Unable to determine log type from filename (%s); try -i?\n", path);
        }
        usage();
        exit(1);
    }

    switch (log_format) {
    case log_format_tlog:
        prep_for_tlog();
        break;
    case log_format_df:
        prep_for_df();
        break;
    case log_format_log:
        prep_for_log();
        break;
    default:
        abort();
    }

    int fd;
    ssize_t fd_size = -1;
    if (streq(path, "-")) {
        // fd = fileno(stdin);  // doesn't work on Cygwin
        fd = 0;
    } else {
        fd = xopen(path, O_RDONLY);

        struct stat buf;
        if (fstat(fd, &buf) == -1) {
            ::fprintf(stderr, "fstat failed: %s\n", strerror(errno));
        } else {
            fd_size = buf.st_size;
        }
    }

    parse_fd(reader, fd, fd_size);

    if (!streq(path, "-")) {
        close(fd);
    }

    switch (log_format) {
    case log_format_tlog:
        cleanup_after_tlog();
        break;
    case log_format_df:
        cleanup_after_df();
        break;
    case log_format_log:
        cleanup_after_log();
        break;
    default:
        abort();
    }
}
int ImageTagger::xopen(const char *filepath, const uint8_t mode)
{
    int fd = open(filepath, mode);
    if (fd == -1) {
        fprintf(stderr, "Failed to open (%s): %s\n", filepath, strerror(errno));
        exit(1);
    }
    return fd;
}

void ImageTagger::prep_for_tlog()
{
    reader = new MAVLink_Reader(config());
    ((MAVLink_Reader*)reader)->set_is_tlog(true);
}

void ImageTagger::prep_for_df()
{
    reader = new DataFlash_Reader(config());
}

void ImageTagger::prep_for_log()
{
    reader = new DataFlash_TextDump_Reader(config());
}

void ImageTagger::cleanup_after_tlog()
{
    reader->clear_message_handlers();
}
void ImageTagger::cleanup_after_df()
{
    reader->clear_message_handlers();
}
void ImageTagger::cleanup_after_log()
{
    reader->clear_message_handlers();
}
// end copied in from loganalyzer.py

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
    init_config();

    la_log(LOG_INFO, "ImageTagger starting: built " __DATE__ " " __TIME__);
    // signal(SIGHUP, sighup_handler);

    reader = new MAVLink_Reader(config());

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

    MH_ImageTagger *tagger = new MH_ImageTagger(images, _time_offset);
    if (tagger != NULL) {
        reader->add_message_handler(tagger, "Tagger");
    } else {
        la_log(LOG_ERR, "Failed to create tagger");
        abort();
    }

    parse_path(_pathname);

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
