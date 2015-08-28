#include "util.h"
#include "analyzer_util.h"
#include "INIReader.h"

class Common_Tool {
public:
    Common_Tool() :
        config_filename(default_config_filename) {
    }

protected:
    class INIReader *_config;
    class INIReader *get_config();
    const char *default_config_filename = "/etc/sololink.conf";
    const char * config_filename;

};
