#include "analyzer_brownout.h"

#include <syslog.h>
#include <stdio.h>
#include <unistd.h> // for fork()
#include "util.h"
#include "analyzer_util.h"

bool Analyzer_Brownout::configure(INIReader *config) {
    if (!MAVLink_Message_Handler::configure(config)) {
	return false;
    }
    return true;
}

void Analyzer_Brownout::results_json_results(Json::Value &root)
{
    if (_vehicle->pos().alt_modtime() > 0) {
        Json::Value result(Json::objectValue);
        result["timestamp"] = 0;
        Json::Value reason(Json::arrayValue);

        const float last_altitude = _vehicle->pos().alt();
        if (last_altitude > max_last_altitude &&
            _vehicle->is_flying()) {
            uint8_t this_sin_score = 10;
            result["status"] = "FAILED";
            result["evilness"] = this_sin_score;
            add_evilness(this_sin_score);
            reason.append(string_format("Possible brownout detected (end of log at %f metres, still flying)", last_altitude));
        } else {
            result["status"] = "OK";
            reason.append(string_format("No brownout detected (final altitude %f metres", last_altitude));
        }
        result["reason"] = reason;
        root.append(result);
    } else {
        uint8_t this_sin_score = 5;
        Json::Value result(Json::objectValue);
        // result["timestamp"] = 0;
        result["status"] = "WARN";
        Json::Value reason(Json::arrayValue);
        reason.append("No VFR_HUD messages received");
        result["evilness"] = this_sin_score;
        add_evilness(this_sin_score);
        result["reason"] = reason;
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
