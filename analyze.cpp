#include "analyze.h"

#include <syslog.h>
#include <stdlib.h> // for exit() (fixme)

#include "analyzer_any_parameters_seen.h"
#include "analyzer_arming_checks.h"
#include "analyzer_altitude_estimate_divergence.h"
#include "analyzer_attitude_estimate_divergence.h"
#include "analyzer_attitude_control.h"
#include "analyzer_battery.h"
#include "analyzer_brownout.h"
#include "analyzer_compass_offsets.h"
#include "analyzer_ever_armed.h"
#include "analyzer_good_ekf.h"
#include "analyzer_notcrashed.h"
#include "analyzer_position_estimate_divergence.h"
#include "analyzer_sensor_health.h"
#include "analyzer_vehicle_definition.h"

void Analyze::instantiate_analyzers(INIReader *config)
{
    if (MAX_ANALYZERS - next_analyzer < 2) {
	syslog(LOG_INFO, "Insufficient analyzer slots (MAX=%d) (next=%d)?!", MAX_ANALYZERS, next_analyzer);
	exit(1);  // FIXME - throw exception
    }

    Analyzer_Any_Parameters_Seen *analyzer_any_parameters_seen = new Analyzer_Any_Parameters_Seen(vehicle,_data_sources);
    if (analyzer_any_parameters_seen != NULL) {
        configure_analyzer(config, analyzer_any_parameters_seen, "Analyzer_Any_Parameters_Seen");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_any_parameters_seen");
    }

    Analyzer_Arming_Checks *analyzer_arming_checks = new Analyzer_Arming_Checks(vehicle,_data_sources);
    if (analyzer_arming_checks != NULL) {
        configure_analyzer(config, analyzer_arming_checks, "Analyzer_Arming_Checks");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_arming_checks");
    }

    analyzer_altitude_estimate_divergence = new Analyzer_Altitude_Estimate_Divergence(vehicle,_data_sources);
    if (analyzer_altitude_estimate_divergence != NULL) {
        configure_analyzer(config, analyzer_altitude_estimate_divergence, "Analyzer_Altitude_Estimate_Divergence");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_altitude_estimate_divergence");
    }

    Analyzer_Attitude_Estimate_Divergence *analyzer_attitude_estimate_divergence = new Analyzer_Attitude_Estimate_Divergence(vehicle,_data_sources);
    if (analyzer_attitude_estimate_divergence != NULL) {
        configure_analyzer(config, analyzer_attitude_estimate_divergence, "Analyzer_Attitude_Estimate_Divergence");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_attitude_estimate_divergence");
    }

    {
        Analyzer_Compass_Offsets *analyzer_compass_offsets = new Analyzer_Compass_Offsets(vehicle,_data_sources, "");
        if (analyzer_compass_offsets != NULL) {
            configure_analyzer(config, analyzer_compass_offsets, "Analyzer_Compass_Offsets");
        } else {
            syslog(LOG_INFO, "Failed to create analyzer_compass_offsets");
        }
    }
    {
        Analyzer_Compass_Offsets *analyzer_compass_offsets = new Analyzer_Compass_Offsets(vehicle,_data_sources, "2");
        if (analyzer_compass_offsets != NULL) {
            configure_analyzer(config, analyzer_compass_offsets, "Analyzer_Compass_Offsets");
        } else {
            syslog(LOG_INFO, "Failed to create analyzer_compass_offsets");
        }
    }
    {
        Analyzer_Compass_Offsets *analyzer_compass_offsets = new Analyzer_Compass_Offsets(vehicle,_data_sources, "3");
        if (analyzer_compass_offsets != NULL) {
            configure_analyzer(config, analyzer_compass_offsets, "Analyzer_Compass_Offsets");
        } else {
            syslog(LOG_INFO, "Failed to create analyzer_compass_offsets");
        }
    }

    Analyzer_Ever_Armed *analyzer_ever_armed = new Analyzer_Ever_Armed(vehicle,_data_sources);
    if (analyzer_ever_armed != NULL) {
        configure_analyzer(config, analyzer_ever_armed, "Analyzer_Ever_Armed");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_ever_armed");
    }

    analyzer_ever_flew = new Analyzer_Ever_Flew(vehicle,_data_sources);
    if (analyzer_ever_flew != NULL) {
        configure_analyzer(config, analyzer_ever_flew, "Analyzer_Ever_Flew");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_ever_flew");
    }


    Analyzer_Good_EKF *analyzer_good_ekf = new Analyzer_Good_EKF(vehicle,_data_sources);
    if (analyzer_good_ekf != NULL) {
        configure_analyzer(config, analyzer_good_ekf, "Analyzer_Good_EKF");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_good_ekf");
    }

    Analyzer_Attitude_Control *analyzer_attitude_control = new Analyzer_Attitude_Control(vehicle,_data_sources);
    if (analyzer_attitude_control != NULL) {
        configure_analyzer(config, analyzer_attitude_control, "Analyzer_Attitude_Control");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_attitude_control");
    }

    Analyzer_Battery *analyzer_battery = new Analyzer_Battery(vehicle,_data_sources);
    if (analyzer_battery != NULL) {
        configure_analyzer(config, analyzer_battery, "Analyzer_Battery");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_battery");
    }

    Analyzer_Brownout *analyzer_brownout = new Analyzer_Brownout(vehicle,_data_sources);
    if (analyzer_brownout != NULL) {
        configure_analyzer(config, analyzer_brownout, "Analyzer_Brownout");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_brownout");
    }

    analyzer_position_estimate_divergence = new Analyzer_Position_Estimate_Divergence(vehicle,_data_sources);
    if (analyzer_position_estimate_divergence != NULL) {
        configure_analyzer(config, analyzer_position_estimate_divergence, "Analyzer_Position_Estimate_Divergence");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_position_estimate_divergence");
    }

    Analyzer_NotCrashed *analyzer_notcrashed = new Analyzer_NotCrashed(vehicle,_data_sources);
    if (analyzer_notcrashed != NULL) {
        configure_analyzer(config, analyzer_notcrashed, "Analyzer_NotCrashed");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_not_crashed");
    }

    Analyzer_Sensor_Health *analyzer_sensor_health = new Analyzer_Sensor_Health(vehicle,_data_sources);
    if (analyzer_sensor_health != NULL) {
        configure_analyzer(config, analyzer_sensor_health, "Analyzer_Sensor_Health");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_sensor_health");
    }

    Analyzer_Vehicle_Definition *analyzer_vehicle_definition = new Analyzer_Vehicle_Definition(vehicle,_data_sources);
    if (analyzer_vehicle_definition != NULL) {
        configure_analyzer(config, analyzer_vehicle_definition, "Analyzer_Vehicle_Definition");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_vehicle_definition");
    }

}


void Analyze::configure_analyzer(INIReader *config,
                                 Analyzer *handler,
                                 const char *handler_name)
{
    if (handler->configure(config)) {
        analyzer[next_analyzer++] = handler;
    } else {
        syslog(LOG_INFO, "Failed to configure (%s)", handler_name);
    }
}

void results_json_add_version(Json::Value &root)
{
    const char *git_version = GIT_VERSION;
    if (git_version != NULL) {
        root["git_version"] = git_version;
    }
}

void Analyze::results_json_add_statistics(Json::Value &root)
{
    if (analyzer_ever_flew != NULL) {
        root["total-flight-time"] = analyzer_ever_flew->total_flight_time() / 1000000.0f;
        root["total-flight-time-units"] = "seconds";
    }
    if (analyzer_position_estimate_divergence != NULL) {
        root["total-distance-travellled"] = analyzer_position_estimate_divergence->total_distance_travelled();
        root["total-distance-travelled-units"] = "metres";
    }
    if (analyzer_altitude_estimate_divergence != NULL) {
        root["maximum-altitude-absolute"] = analyzer_altitude_estimate_divergence->maximum_altitude();
        root["maximum-altitude-absolute-units"] = "metres";
        root["maximum-altitude-relative"] = analyzer_altitude_estimate_divergence->maximum_altitude_relative();
        root["maximum-altitude-relative-units"] = "metres";
    }
}

void Analyze::results_json(Json::Value &root)
{
    Json::Value tests;
    uint16_t total_evilness= 0;
    for(int i=0; i<next_analyzer; i++) {
        const std::string name = analyzer[i]->name();
        if (tests[name].isNull()) {
            Json::Value test_info(Json::objectValue);
            test_info["description"] = analyzer[i]->description();
            test_info["name"] = name;
            test_info["results"] = Json::Value(Json::arrayValue);
            tests[name] = test_info;
        }

        analyzer[i]->results_json_results(tests[name]["results"]);
        tests[name]["evilness"] = tests[name]["evilness"].asLargestUInt() + analyzer[i]->evilness();
        tests[name]["severity-score"] = tests[name]["evilness"];
        total_evilness += analyzer[i]->evilness();
    }
    
    root["evilness"] = total_evilness;
    root["severity-score"] = root["evilness"];
    root["tests"] = tests;
    results_json_add_version(root);

    results_json_add_statistics(root);
}


namespace Json {
    class PlainTextWriter : public Json::Writer {
    public:
        PlainTextWriter() :
            depth(0)
            { }
        std::string write( const Value &root );
        void writeValue(std::string &document, const Value &value);
    private:
        uint16_t depth;
        void writeIndent(std::string &document);
    };

    std::string PlainTextWriter::write( const Value &root )
    {
        std::string document = "";
        depth = 0;
        writeValue(document, root);
        return document;
    }
    void PlainTextWriter::writeIndent(std::string &document)
    {
        // FIXME, surely!
        for (uint8_t i=0; i< depth; i++) {
            document += "    ";
        }
    }

    void PlainTextWriter::writeValue(std::string &document, const Value &value)
    {
        switch(value.type()) {
        case nullValue:
            break;
        case intValue:
            document += valueToString(value.asLargestInt());
            break;
        case uintValue:
            document += valueToString(value.asLargestUInt());
            break;
        case realValue:
            document += valueToString(value.asDouble());
            break;
        case stringValue:
            document += valueToQuotedString(value.asString().c_str());
            break;
        case booleanValue:
            document += value.asBool();
            break;
        case arrayValue: {
            for (uint8_t index = 0; index < value.size(); index++) {
                switch(value[index].type()) {
                case nullValue:
                case intValue:
                case uintValue:
                case realValue:
                case stringValue:
                case booleanValue:
                    writeIndent(document);
                    writeValue(document, value[index]);
                    document += "\n";
                    break;
                case arrayValue:
                case objectValue:
                    depth++;
                    writeValue(document, value[index]);
                    depth--;
                    break;
                }
            }
            break;
        }
        case objectValue: {
            Value::Members members(value.getMemberNames());
            for (Value::Members::iterator it = members.begin();
                 it != members.end(); ++it) {
                const std::string &name = *it;
                writeIndent(document);
                document += name;
                document += ":";
                switch(value[name].type()) {
                case nullValue:
                case intValue:
                case uintValue:
                case realValue:
                case stringValue:
                case booleanValue:
                    document += " ";
                    writeValue(document, value[name]);
                    document += "\n";
                    break;
                case arrayValue:
                case objectValue:
                    document += "\n";
                    depth++;
                    writeValue(document, value[name]);
                    depth--;
                }
            }
            break;
        }
        }
    }
}


namespace Json {
    class HTMLWriter : public Json::Writer {
    public:
        HTMLWriter() :
            depth(0)
            { }
        std::string write( const Value &root );
        void writeValue(std::string &document, const Value &value);
    private:
        uint16_t depth;
        void writeIndent(std::string &document);
    };

    std::string HTMLWriter::write( const Value &root )
    {
        std::string document = "";
        depth = 0;
        writeValue(document, root);
        return document;
    }
    void HTMLWriter::writeIndent(std::string &document)
    {
        // FIXME, surely!
        for (uint8_t i=0; i< depth; i++) {
            document += "    ";
        }
    }

    void HTMLWriter::writeValue(std::string &document, const Value &value)
    {
        switch(value.type()) {
        case nullValue:
            break;
        case intValue:
            document += valueToString(value.asLargestInt());
            break;
        case uintValue:
            document += valueToString(value.asLargestUInt());
            break;
        case realValue:
            document += valueToString(value.asDouble());
            break;
        case stringValue:
            document += valueToQuotedString(value.asString().c_str());
            break;
        case booleanValue:
            document += value.asBool();
            break;
        case arrayValue: {
            document += "<ol>";
            for (uint8_t index = 0; index < value.size(); index++) {
                switch(value[index].type()) {
                case nullValue:
                case intValue:
                case uintValue:
                case realValue:
                case stringValue:
                case booleanValue:
                    writeIndent(document);
                    document += "<li>";
                    writeValue(document, value[index]);
                    document += "</li>";
                    document += "\n";
                    break;
                case arrayValue:
                case objectValue:
                    depth++;
                    document += "<li>";
                    writeValue(document, value[index]);
                    depth--;
                    document += "</li>";
                    break;
                }
            }
            break;
        }
        case objectValue: {
            Value::Members members(value.getMemberNames());
            if (depth == 0) {
                document += "<dl class='la_results'>";
            } else {
                document += "<dl>";
            }
            for (Value::Members::iterator it = members.begin();
                 it != members.end(); ++it) {
                const std::string &name = *it;
                writeIndent(document);
                document += "<dt>";
                document += name;
                document += "</dt>\n";
                writeIndent(document);
                document += "<dd>";
                switch(value[name].type()) {
                case nullValue:
                case intValue:
                case uintValue:
                case realValue:
                case stringValue:
                case booleanValue:
                    writeValue(document, value[name]);
                    break;
                case arrayValue:
                case objectValue:
                    depth++;
                    writeValue(document, value[name]);
                    depth--;
                }
                document += "</dd>\n";
            }
            document += "</dl>\n";
            break;
        }
        }
    }
}

void Analyze::end_of_log(uint32_t packet_count) {
    for(int i=0; i<next_analyzer; i++) {
        analyzer[i]->end_of_log(packet_count);
    }

    Json::Value root;
    root["format-version"] = "0.1";
    root["timestamp"] = (Json::UInt64)start_time;
    root["duration"] = (Json::UInt64)(now() - start_time);

    results_json(root);

    root["packet_count"] = packet_count;

    Json::Writer *writer;
    switch(_output_style) {
    case OUTPUT_JSON: {
        writer = new Json::StyledWriter();
        break;
    }
    case OUTPUT_PLAINTEXT:
        writer = new Json::PlainTextWriter();
        break;
    case OUTPUT_HTML:
        writer = new Json::HTMLWriter();
        break;
    }
    fprintf(stdout, "%s", writer->write(root).c_str());
}


void Analyze::evaluate_all() {
    for(int i=0; i<next_analyzer; i++) {
        analyzer[i]->evaluate();
    }
}



