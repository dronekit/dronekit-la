#ifndef _DATA_SOURCES_H
#define _DATA_SOURCES_H

#include <vector>

class Data_Source {
public:
    std::vector<std::string> series() const { return _series; }
    void add_series(std::string series) {
        _series.push_back(series);
    }
private:
    std::vector<std::string> _series;
};

class Data_Sources {
public:
    // FIXME: use an enum for type here
    void add_series(const std::string type, std::string data_source) {
        _get(type)->add_series(data_source);
    }
    const Data_Source* get(std::string type) {
        return _get(type);
    }
    
private:
    Data_Source* _get(const std::string type) {
        // if (_data_sources.count(type) == 0) {
        //     ::fprintf(stderr, "Asked for data source (%s) which does not exist\n", type.c_str());
        // }
        if (_data_sources[type] == NULL) {
            _data_sources[type] = new Data_Source();
        }
        return _data_sources[type];
    }
    std::map<const std::string, Data_Source*> _data_sources;
};

#endif
