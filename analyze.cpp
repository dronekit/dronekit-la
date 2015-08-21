#include "analyze.h"

#include <syslog.h>
#include <stdlib.h> // for exit() (fixme)

#include "analyzer_compass_offsets.h"
#include "analyzer_ever_armed.h"
#include "analyzer_ever_flew.h"
#include "analyzer_good_ekf.h"
#include "analyzer_battery.h"
#include "analyzer_brownout.h"
#include "analyzer_crashed.h"

void Analyze::instantiate_analyzers(INIReader *config)
{
    if (MAX_ANALYZERS - next_analyzer < 2) {
	syslog(LOG_INFO, "Insufficient analyzer slots (MAX=%d) (next=%d)?!", MAX_ANALYZERS, next_analyzer);
	exit(1);  // FIXME - throw exception
    }

    Analyzer_Compass_Offsets *analyzer_compass_offsets = new Analyzer_Compass_Offsets(_fd_telem_forwarder, _sa_telemetry_forwarder);
    if (analyzer_compass_offsets != NULL) {
        configure_analyzer(config, analyzer_compass_offsets, "Analyzer_Compass_Offsets");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_compass_offsets");
    }

    Analyzer_Ever_Armed *analyzer_ever_armed = new Analyzer_Ever_Armed(_fd_telem_forwarder, _sa_telemetry_forwarder);
    if (analyzer_ever_armed != NULL) {
        configure_analyzer(config, analyzer_ever_armed, "Analyzer_Ever_Armed");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_ever_armed");
    }

    Analyzer_Ever_Flew *analyzer_ever_flew = new Analyzer_Ever_Flew(_fd_telem_forwarder, _sa_telemetry_forwarder);
    if (analyzer_ever_flew != NULL) {
        configure_analyzer(config, analyzer_ever_flew, "Analyzer_Ever_Flew");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_ever_flew");
    }


    Analyzer_Good_EKF *analyzer_good_ekf = new Analyzer_Good_EKF(_fd_telem_forwarder, _sa_telemetry_forwarder);
    if (analyzer_good_ekf != NULL) {
        configure_analyzer(config, analyzer_good_ekf, "Analyzer_Good_EKF");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_good_ekf");
    }

    Analyzer_Battery *analyzer_battery = new Analyzer_Battery(_fd_telem_forwarder, _sa_telemetry_forwarder);
    if (analyzer_battery != NULL) {
        configure_analyzer(config, analyzer_battery, "Analyzer_Battery");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_battery");
    }

    Analyzer_Brownout *analyzer_brownout = new Analyzer_Brownout(_fd_telem_forwarder, _sa_telemetry_forwarder);
    if (analyzer_brownout != NULL) {
        configure_analyzer(config, analyzer_brownout, "Analyzer_Brownout");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_brownout");
    }


    Analyzer_Crashed *analyzer_crashed = new Analyzer_Crashed(_fd_telem_forwarder, _sa_telemetry_forwarder);
    if (analyzer_crashed != NULL) {
        configure_analyzer(config, analyzer_crashed, "Analyzer_Crashed");
    } else {
        syslog(LOG_INFO, "Failed to create analyzer_crashed");
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
uint32_t Analyze::results_json(char *buf, const uint32_t buflen)
{
    Json::Value root;
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

    Json::StyledWriter writer;
    std::string output = writer.write(root);
    const char * string = output.c_str();
    strncpy(buf, string, buflen);
    return strlen(buf);
}

void Analyze::end_of_log() {
    for(int i=0; i<next_analyzer; i++) {
        analyzer[i]->end_of_log();
    }

    char buffer[102480];
    uint32_t used = results_json(buffer,10240);
    buffer[used] = '\0';
    fprintf(stdout, "%s", buffer);

}

void Analyze::handle_decoded_message(uint64_t T, mavlink_ahrs2_t &msg) {
    for(int i=0; i<next_analyzer; i++) {
        analyzer[i]->handle_decoded_message(T, msg);
    }
}
void Analyze::handle_decoded_message(uint64_t T, mavlink_attitude_t &msg) {
    for(int i=0; i<next_analyzer; i++) {
        analyzer[i]->handle_decoded_message(T, msg);
    }
}
void Analyze::handle_decoded_message(uint64_t T, mavlink_heartbeat_t &msg) {
    for(int i=0; i<next_analyzer; i++) {
        analyzer[i]->handle_decoded_message(T, msg);
    }
}
void Analyze::handle_decoded_message(uint64_t T, mavlink_param_value_t &msg) {
    for(int i=0; i<next_analyzer; i++) {
        analyzer[i]->handle_decoded_message(T, msg);
    }
}
void Analyze::handle_decoded_message(uint64_t T, mavlink_servo_output_raw_t &msg) {
    for(int i=0; i<next_analyzer; i++) {
        analyzer[i]->handle_decoded_message(T, msg);
    }
}
void Analyze::handle_decoded_message(uint64_t T, mavlink_ekf_status_report_t &msg) {
    for(int i=0; i<next_analyzer; i++) {
        analyzer[i]->handle_decoded_message(T, msg);
    }
}
void Analyze::handle_decoded_message(uint64_t T, mavlink_sys_status_t &msg) {
    for(int i=0; i<next_analyzer; i++) {
        analyzer[i]->handle_decoded_message(T, msg);
    }
}
void Analyze::handle_decoded_message(uint64_t T, mavlink_vfr_hud_t &msg) {
    for(int i=0; i<next_analyzer; i++) {
        analyzer[i]->handle_decoded_message(T, msg);
    }
}
