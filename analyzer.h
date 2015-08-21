#ifndef ANALYZER_H
#define ANALYZER_H

#include "mavlink_message_handler.h"

#include <jsoncpp/json/json.h> // libjsoncpp0 and libjsoncpp-dev on debian
#include <jsoncpp/json/writer.h> // libjsoncpp0 and libjsoncpp-dev on debian

class Analyzer : public MAVLink_Message_Handler {

public:
    Analyzer(int fd, struct sockaddr_in &sa) :
	MAVLink_Message_Handler(fd, sa),
        evilness(0)
        { }

    virtual const char *name() = 0;
    virtual const char *description() = 0;
    virtual void results_json_results(Json::Value &root) = 0;
    virtual void end_of_log() { }

    void add_evilness(uint8_t sin_points) {
        evilness += sin_points;
    }
    uint16_t get_evilness() { return evilness; }

protected:
    std::string to_string(double x);
    uint16_t evilness;

private:
};

#endif
    
