#ifndef ANALYZE_H
#define ANALYZE_H

#include "mavlink_message_handler.h"

#include "analyzer.h"
#include "analyzervehicle_copter.h"

#include "analyzer_util.h"

#include "analyzer_altitude_estimate_divergence.h"
#include "analyzer_ever_flew.h"
#include "analyzer_position_estimate_divergence.h"

#include "data_sources.h"

class Analyze {

public:
    Analyze(AnalyzerVehicle::Base *&vehicle) :
        vehicle(vehicle)
        {
            start_time = now();
        }
    void instantiate_analyzers(INIReader *config);

    void end_of_log(uint32_t packet_count);

    enum output_style_option {
        OUTPUT_JSON = 17,
        OUTPUT_PLAINTEXT,
        OUTPUT_HTML,
        OUTPUT_BRIEF,
    };

    void set_output_style(output_style_option option) { _output_style = option;}
    output_style_option output_style() { return _output_style; }

    void evaluate_all();

    void add_data_source(std::string type, const char *data_source) {
        _data_sources.add(type, data_source);
    }

    void add_data_source(std::string type, const char *data_source) {
        _data_sources.add(type, data_source);
    }

    std::vector<Analyzer *> analyzers() { return _analyzers; }

    void set_analyzer_names_to_run(const std::vector<std::string> run_these);

protected:

private:
    uint64_t start_time;

    AnalyzerVehicle::Base *&vehicle;

    output_style_option _output_style = OUTPUT_JSON;

    std::vector<Analyzer*> _analyzers;

    bool _use_names_to_run = false;
    std::map<std::string,bool> _names_to_run;
    
    void configure_message_handler(INIReader *config,
                                   MAVLink_Message_Handler *handler,
                                   const char *handler_name);

    void configure_analyzer(INIReader *config, Analyzer *handler);

    void set_vehicle_copter();
    void set_copter_frametype(const char *frame_config_string);

    void results_json_add_statistics(Json::Value &root);
    void results_json(Json::Value &root);

    void output_plaintext(Json::Value &root);

    Data_Sources _data_sources;

    Analyzer_Altitude_Estimate_Divergence *analyzer_altitude_estimate_divergence = NULL;    
    Analyzer_Ever_Flew *analyzer_ever_flew = NULL;
    Analyzer_Position_Estimate_Divergence *analyzer_position_estimate_divergence = NULL;    
};

#endif
    
