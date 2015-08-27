#include "analyze.h"

#include <syslog.h>
#include <stdlib.h> // for exit() (fixme)

#include "analyzer_compass_offsets.h"
#include "analyzer_ever_armed.h"
#include "analyzer_ever_flew.h"
#include "analyzer_good_ekf.h"
#include "analyzer_attitude_control.h"
#include "analyzer_battery.h"
#include "analyzer_brownout.h"
#include "analyzer_notcrashed.h"

void Analyze::instantiate_analyzers(INIReader *config)
{
    if (MAX_ANALYZERS - next_analyzer < 2) {
	syslog(LOG_INFO, "Insufficient analyzer slots (MAX=%d) (next=%d)?!", MAX_ANALYZERS, next_analyzer);
	exit(1);  // FIXME - throw exception
    }

    Analyzer_Compass_Offsets *analyzer_compass_offsets = new Analyzer_Compass_Offsets(_fd_telem_forwarder, _sa_telemetry_forwarder, vehicle);
    if (analyzer_compass_offsets != NULL) {
        configure_analyzer(config, analyzer_compass_offsets, "Analyzer_Compass_Offsets");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_compass_offsets");
    }

    Analyzer_Ever_Armed *analyzer_ever_armed = new Analyzer_Ever_Armed(_fd_telem_forwarder, _sa_telemetry_forwarder, vehicle);
    if (analyzer_ever_armed != NULL) {
        configure_analyzer(config, analyzer_ever_armed, "Analyzer_Ever_Armed");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_ever_armed");
    }

    Analyzer_Ever_Flew *analyzer_ever_flew = new Analyzer_Ever_Flew(_fd_telem_forwarder, _sa_telemetry_forwarder, vehicle);
    if (analyzer_ever_flew != NULL) {
        configure_analyzer(config, analyzer_ever_flew, "Analyzer_Ever_Flew");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_ever_flew");
    }


    Analyzer_Good_EKF *analyzer_good_ekf = new Analyzer_Good_EKF(_fd_telem_forwarder, _sa_telemetry_forwarder, vehicle);
    if (analyzer_good_ekf != NULL) {
        configure_analyzer(config, analyzer_good_ekf, "Analyzer_Good_EKF");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_good_ekf");
    }

    Analyzer_Attitude_Control *analyzer_attitude_control = new Analyzer_Attitude_Control(_fd_telem_forwarder, _sa_telemetry_forwarder, vehicle);
    if (analyzer_attitude_control != NULL) {
        configure_analyzer(config, analyzer_attitude_control, "Analyzer_Attitude_Control");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_attitude_control");
    }

    Analyzer_Battery *analyzer_battery = new Analyzer_Battery(_fd_telem_forwarder, _sa_telemetry_forwarder, vehicle);
    if (analyzer_battery != NULL) {
        configure_analyzer(config, analyzer_battery, "Analyzer_Battery");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_battery");
    }

    Analyzer_Brownout *analyzer_brownout = new Analyzer_Brownout(_fd_telem_forwarder, _sa_telemetry_forwarder, vehicle);
    if (analyzer_brownout != NULL) {
        configure_analyzer(config, analyzer_brownout, "Analyzer_Brownout");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_brownout");
    }


    Analyzer_NotCrashed *analyzer_notcrashed = new Analyzer_NotCrashed(_fd_telem_forwarder, _sa_telemetry_forwarder, vehicle);
    if (analyzer_notcrashed != NULL) {
        configure_analyzer(config, analyzer_notcrashed, "Analyzer_NotCrashed");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_not_crashed");
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
void Analyze::results_json(Json::Value &root)
{
    Json::Value tests;
    uint16_t total_evilness= 0;
    for(int i=0; i<next_analyzer; i++) {
        Json::Value results(Json::arrayValue);
        analyzer[i]->results_json_results(results);
        uint16_t evilness = analyzer[i]->get_evilness();
        total_evilness += evilness;
        const char *name = analyzer[i]->name();

        Json::Value test_info(Json::objectValue);
        test_info["description"] = analyzer[i]->description();
        test_info["results"] = results;
        test_info["evilness"] = evilness;
        test_info["name"] = name;

        tests[name] = test_info;
    }
    
    root["evilness"] = total_evilness;
    root["tests"] = tests;
    results_json_add_version(root);
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

void Analyze::handle_decoded_message(uint64_t T, mavlink_ahrs2_t &msg) {
    if (!vehicle) {
        return;
    }
    for(int i=0; i<next_analyzer; i++) {
        analyzer[i]->handle_decoded_message(T, msg);
    }
}
void Analyze::handle_decoded_message(uint64_t T, mavlink_attitude_t &msg) {
    if (!vehicle) {
        return;
    }
    vehicle->handle_decoded_message(T, msg);
    for(int i=0; i<next_analyzer; i++) {
        analyzer[i]->handle_decoded_message(T, msg);
    }
}
void Analyze::handle_decoded_message(uint64_t T, mavlink_heartbeat_t &msg) {
    if (!vehicle) {
        return;
    }
    vehicle->handle_decoded_message(T, msg);
    for(int i=0; i<next_analyzer; i++) {
        analyzer[i]->handle_decoded_message(T, msg);
    }
}
void Analyze::handle_decoded_message(uint64_t T, mavlink_nav_controller_output_t &msg) {
    if (!vehicle) {
        return;
    }
    vehicle->handle_decoded_message(T, msg);
    for(int i=0; i<next_analyzer; i++) {
        analyzer[i]->handle_decoded_message(T, msg);
    }
}
void Analyze::handle_decoded_message(uint64_t T, mavlink_param_value_t &msg) {
    if (!vehicle) {
        return;
    }
    vehicle->handle_decoded_message(T, msg);
    for(int i=0; i<next_analyzer; i++) {
        analyzer[i]->handle_decoded_message(T, msg);
    }
}
void Analyze::handle_decoded_message(uint64_t T, mavlink_servo_output_raw_t &msg) {
    if (!vehicle) {
        return;
    }
    vehicle->handle_decoded_message(T, msg);
    for(int i=0; i<next_analyzer; i++) {
        analyzer[i]->handle_decoded_message(T, msg);
    }
}
void Analyze::handle_decoded_message(uint64_t T, mavlink_ekf_status_report_t &msg) {
    if (!vehicle) {
        return;
    }
    for(int i=0; i<next_analyzer; i++) {
        analyzer[i]->handle_decoded_message(T, msg);
    }
}
void Analyze::handle_decoded_message(uint64_t T, mavlink_sys_status_t &msg) {
    if (!vehicle) {
        return;
    }
    for(int i=0; i<next_analyzer; i++) {
        analyzer[i]->handle_decoded_message(T, msg);
    }
}
void Analyze::handle_decoded_message(uint64_t T, mavlink_vfr_hud_t &msg) {
    if (!vehicle) {
        return;
    }
    for(int i=0; i<next_analyzer; i++) {
        analyzer[i]->handle_decoded_message(T, msg);
    }
}
void Analyze::handle_decoded_message(uint64_t T, mavlink_statustext_t &msg) {
    if (!vehicle) {
        if (strstr(msg.text, "APM:Copter")) {
            vehicle = new AnalyzerVehicle::Copter();
        }
    }
    if (!vehicle) {
        return;
    }
    vehicle->handle_decoded_message(T, msg);
    for(int i=0; i<next_analyzer; i++) {
        analyzer[i]->handle_decoded_message(T, msg);
    }
}
