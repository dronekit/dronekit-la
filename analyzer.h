#ifndef ANALYZER_H
#define ANALYZER_H

#include <jsoncpp/json/json.h> // libjsoncpp0 and libjsoncpp-dev on debian
#include <jsoncpp/json/writer.h> // libjsoncpp0 and libjsoncpp-dev on debian

#include "analyzervehicle.h"
#include "INIReader.h"

enum analyzer_status {
    analyzer_status_warn = 17,
    analyzer_status_fail,
    analyzer_status_ok,
};

class analyzer_result {
public:
    analyzer_status status;
    const char *status_as_string() const {
        switch(status) {
        case analyzer_status_fail:
            return "FAIL";
        case analyzer_status_warn:
            return "WARN";
        case analyzer_status_ok:
            return "OK";
        }
        return "STRANGE";
    }
};


class Analyzer {

public:
    Analyzer(AnalyzerVehicle::Base *&vehicle) :
        _vehicle(vehicle)
        { }

    virtual bool configure(INIReader *config) {
        return true;
    }

    virtual const char *name() const = 0;
    virtual const char *description() const = 0;
    virtual void results_json_results(Json::Value &root) = 0;
    virtual void end_of_log(uint32_t packet_count) { }

    void add_evilness(uint8_t sin_points) {
        evilness += sin_points;
    }
    uint16_t get_evilness() const { return evilness; }

    analyzer_status status() const { return _status; }

    virtual void evaluate() { }
protected:
    std::string to_string(double x);

    AnalyzerVehicle::Base *&_vehicle;

    uint16_t evilness = 0;
    void set_status(analyzer_status status) { _status = status; }

private:
    analyzer_status _status = analyzer_status_ok;
};

#endif
    


// - two fundamental types of test
//  - is the software working correctly (EKF issues)
//  - is the vehicle doing sensible things (attitude etc)
