#ifndef _DATA_SOURCES_H
#define _DATA_SOURCES_H

#include <vector>

class Data_Sources {
public:
    // FIXME: use an enum for type here
    void add(std::string type, std::string data_source) {
        _data_sources[type].push_back(data_source);
    }
    std::vector<std::string> get(std::string type) {
        // if (_data_sources.count(type) == 0) {
        //     ::fprintf(stderr, "Asked for data source which does not exist\n");
        // }
        return _data_sources[type];
    }

private:
    std::map<std::string, std::vector<std::string>> _data_sources;
};

#endif
