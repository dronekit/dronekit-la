#include "analyzer_brownout.h"

#include "util.h"
#include "analyzer_util.h"

bool Analyzer_Brownout::configure(INIReader *config) {
    if (!Analyzer::configure(config)) {
	return false;
    }
    return true;
}

void Analyzer_Brownout::results_json_results(Json::Value &root)
{
    if (!_vehicle) {
        return;
    }
    if (_vehicle->alt_modtime() > 0) {
        Json::Value result(Json::objectValue);
        result["timestamp"] = 0;

        const float last_altitude = _vehicle->alt();
        if (last_altitude > max_last_altitude &&
            _vehicle->is_flying()) {
            uint8_t this_sin_score = 10;
            result["status"] = "FAILED";
            result["evilness"] = this_sin_score;
            add_evilness(this_sin_score);
            result["reason"] = "Log ended while craft still flying";

            Json::Value evidence(Json::arrayValue);
            evidence.append(string_format("Final altitude %f metres", last_altitude));
            evidence.append("Vehicle still flying");
            result["evidence"] = evidence;
        } else {
            result["status"] = "OK";
            result["reason"] = "No brownout detected";

            Json::Value evidence(Json::arrayValue);
            evidence.append(string_format("Final altitude %f metres", last_altitude));
            result["evidence"] = evidence;
        }
        root.append(result);
    } else {
        uint8_t this_sin_score = 5;
        Json::Value result(Json::objectValue);
        // result["timestamp"] = 0;
        result["status"] = "WARN";
        result["reason"] = "No VFR_HUD messages received";
        result["evilness"] = this_sin_score;
        add_evilness(this_sin_score);
        Json::Value series(Json::arrayValue);
        series.append("VFR_HUD.alt");
        series.append("SERVO_OUTPUT_RAW.servo1_raw");
        series.append("SERVO_OUTPUT_RAW.servo2_raw");
        series.append("SERVO_OUTPUT_RAW.servo3_raw");
        series.append("SERVO_OUTPUT_RAW.servo4_raw");
        result["series"] = series;
        root.append(result);
    }
}
