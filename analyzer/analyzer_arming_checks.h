#ifndef ANALYZER_ARMING_CHECKS_H
#define ANALYZER_ARMING_CHECKS_H

/*
 * analyzer_arming_checks
 *
 */

#include "analyzer.h"

class Analyzer_Arming_Checks_Result : public Analyzer_Result_Event {
public:
    void set_arming_check(uint32_t arming_check) { _arming_check = arming_check; }
    uint32_t arming_check() const { return _arming_check; }
private:
    uint32_t _arming_check;
};

class Analyzer_Arming_Checks : public Analyzer {

public:

    Analyzer_Arming_Checks(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
    Analyzer(vehicle,data_sources)
    { }

    const std::string name() const override { return "Arming Checks"; }
    const std::string description() const override {
        return "An autopilot checks many aspects of the aircraft's state before allowing it to be armed - for example, that it has a good GPS fix.  This test will FAIL if the craft ever arms when some arming checks are disabled.";
    }

    void evaluate() override;

private:
    bool _armed = false;

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


