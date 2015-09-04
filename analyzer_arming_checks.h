#ifndef ANALYZER_ARMING_CHECKS_H
#define ANALYZER_ARMING_CHECKS_H

/*
 * analyzer_arming_checks
 *
 */

#include "analyzer.h"

class Analyzer_Arming_Checks : public Analyzer {

public:
    Analyzer_Arming_Checks(int fd, struct sockaddr_in *sa, AnalyzerVehicle::Base *&vehicle) :
	Analyzer(fd, sa, vehicle)
    { }

    const char *name() const override { return "Arming Checks"; }
    const char *description() const override {
        return "This test will FAIL if the craft ever arms when some arming checks are disabled";
    }

    bool configure(INIReader *config);

    void evaluate(uint64_t T);
    void handle_decoded_message(uint64_t T, mavlink_heartbeat_t &heartbeat);
    void handle_decoded_message(uint64_t T, mavlink_param_value_t &param);
    
    void results_json_results(Json::Value &root);

private:
    bool _armed = false;

    struct result {
        uint64_t T;
        uint32_t arming_check;
    };

    #define MAX_RESULTS 100
    uint8_t next_result = 0;
    struct result results[MAX_RESULTS] = { };

    void results_json_results_do_result(Json::Value &root, const struct result result);

    // from AP_Arming.h:
    enum ArmingChecks {
        ARMING_CHECK_NONE       = 0x0000,
        ARMING_CHECK_ALL        = 0x0001,
        ARMING_CHECK_BARO       = 0x0002,
        ARMING_CHECK_COMPASS    = 0x0004,
        ARMING_CHECK_GPS        = 0x0008,
        ARMING_CHECK_INS        = 0x0010,
        ARMING_CHECK_PARAMETERS = 0x0020,
        ARMING_CHECK_RC         = 0x0040,
        ARMING_CHECK_VOLTAGE    = 0x0080,
        ARMING_CHECK_BATTERY    = 0x0100,
        ARMING_CHECK_AIRSPEED   = 0x0200,
        ARMING_CHECK_LOGGING    = 0x0400,
    };

    std::map<uint32_t, const char*> _bit_to_name = {
        { ARMING_CHECK_NONE, "NONE" },
        { ARMING_CHECK_ALL, "ALL" },
        { ARMING_CHECK_BARO, "BARO" },
        { ARMING_CHECK_COMPASS, "COMPASS" },
        { ARMING_CHECK_GPS, "GPS" },
        { ARMING_CHECK_INS, "INS" },
        { ARMING_CHECK_PARAMETERS, "PARAMETERS" },
        { ARMING_CHECK_RC, "RC" },
        { ARMING_CHECK_VOLTAGE, "VOLTAGE" },
        { ARMING_CHECK_BATTERY, "BATTERY" },
        { ARMING_CHECK_AIRSPEED, "AIRSPEED" },
        { ARMING_CHECK_LOGGING, "LOGGING" },
    };

};

#endif


