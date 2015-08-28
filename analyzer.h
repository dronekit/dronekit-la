#ifndef ANALYZER_H
#define ANALYZER_H

#include "mavlink_message_handler.h"

#include <jsoncpp/json/json.h> // libjsoncpp0 and libjsoncpp-dev on debian
#include <jsoncpp/json/writer.h> // libjsoncpp0 and libjsoncpp-dev on debian

#include "analyzervehicle.h"

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
    // std::string reason_as_string() const {
    //     return string_format("Desired attitude not achieved");
    // }
};


class Analyzer : public MAVLink_Message_Handler {

public:
    Analyzer(int fd, struct sockaddr_in &sa, AnalyzerVehicle::Base *&vehicle) :
	MAVLink_Message_Handler(fd, sa),
        evilness(0),
        _vehicle(vehicle),
        _status(analyzer_status_ok)
        { }

    virtual const char *name() = 0;
    virtual const char *description() = 0;
    virtual void results_json_results(Json::Value &root) = 0;
    virtual void end_of_log(uint32_t packet_count) { }

    void add_evilness(uint8_t sin_points) {
        evilness += sin_points;
    }
    uint16_t get_evilness() const { return evilness; }

    analyzer_status status() const { return _status; }

protected:

    std::string to_string(double x);
    uint16_t evilness;
    AnalyzerVehicle::Base *&_vehicle;
    void set_status(analyzer_status status) { _status = status; }

private:
    analyzer_status _status;
};

#endif
    


// - two fundamental types of test
//  - is the software working correctly (EKF issues)
//  - is the vehicle doing sensible things (attitude etc)
