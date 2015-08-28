#include "common_tool.h"

#include <sys/stat.h>
#include <errno.h>

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

