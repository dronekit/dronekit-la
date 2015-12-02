#include "analyzer_attitude_control.h"

#include <stdio.h>
#include "util.h"
#include "analyzer_util.h"

#include "analyzervehicle_copter.h"

bool Analyzer_Attitude_Control::configure(INIReader *config) {
    if (!Analyzer::configure(config)) {
        return false;
    }

    offset_warn = config->GetReal("loganalyzer", "attitude_control::delta_warn", 5.0f);
    offset_fail = config->GetReal("loganalyzer", "attitude_control::delta_fail", 10.0f);
    duration_min = config->GetReal("loganalyzer", "attitude_control::duration_min", 250000);

    return true;
}

void Analyzer_Attitude_Control::open_result(double delta)
{
    _result = new Analyzer_Attitude_Control_Result();
    _result->set_T_start(_vehicle->T());
    _result->set_reason("Desired attitude not achieved");
    _result->add_source(_data_sources.get("ATTITUDE"));
    _result->add_source(_data_sources.get("DESATTITUDE"));

    if (delta > offset_fail) {
        _result->set_status(analyzer_status_fail);
        _result->set_severity_score(10);
    } else if (delta > offset_warn) {
        _result->set_status(analyzer_status_warn);
        _result->set_severity_score(5);
    } else {
        ::fprintf(stderr, "open result for bad delta");
    }
    
    _result->roll_at_deltamax = _vehicle->roll();
    _result->desroll_at_deltamax = _vehicle->desroll();
    _result->pitch_at_deltamax = _vehicle->pitch();
    _result->despitch_at_deltamax = _vehicle->despitch();
}

void Analyzer_Attitude_Control::update_result(const double delta)
{
    if (delta > _result->deltamax) {
        _result->deltamax = delta;
        _result->roll_at_deltamax = _vehicle->roll();
        _result->desroll_at_deltamax = _vehicle->desroll();
        _result->pitch_at_deltamax = _vehicle->pitch();
        _result->despitch_at_deltamax = _vehicle->despitch();
        if (delta > offset_fail) {
            _result->set_status(analyzer_status_fail);
            _result->set_severity_score(10);
        }
    }
    // should probably subclass attitude_control rather than this:
    if (_vehicle->vehicletype() == AnalyzerVehicle::Base::vehicletype_t::copter) {
        AnalyzerVehicle::Copter *&copter = (AnalyzerVehicle::Copter*&)_vehicle;
        std::set<uint8_t> high = copter->motors_clipping_high();
        std::set<uint8_t> low = copter->motors_clipping_low();

        for (uint8_t i=1; i <= copter->num_motors(); i++) {
            if (low.count(i)) {
                _result->motors_clipping_low.insert(i);
            }
        }
        for (uint8_t i=1; i <= copter->num_motors(); i++) {
            if (high.count(i)) {
                _result->motors_clipping_high.insert(i);
            }
        }

        switch (copter->frame_type()) {
        case AnalyzerVehicle::Copter::copter_frame_type::frame_type_quad:
            if (high.count(1) && low.count(2)) {
                _result->motors_failing.insert(1);
            }
            if (high.count(2) && low.count(1)) {
                _result->motors_failing.insert(2);
            }
            if (high.count(3) && low.count(4)) {
                _result->motors_failing.insert(3);
            }
            if (high.count(4) && low.count(3)) {
                _result->motors_failing.insert(4);
            }
            break;
        default:
            break;
        }
    }
}

void Analyzer_Attitude_Control::close_result()
{
    _result->set_T_stop(_vehicle->T());

    if (_result->duration() < duration_min) {
        delete _result;
        _result = NULL; 
        return;
    }
  
    if (_result->deltamax >= offset_fail) {
        _result->set_status(analyzer_status_fail);
        _result->add_evidence(string_format("threshold=%f", offset_fail));
    } else if (_result->deltamax >= offset_warn) {
        _result->set_status(analyzer_status_warn);
        _result->add_evidence(string_format("threshold=%f", offset_warn));
    }

    _result->add_evidence(string_format("threshold-duration=%f seconds", duration_min/1000000.0f));
    _result->add_evidence(string_format("Max-Delta=%f degrees", _result->deltamax));
    _result->add_evidence(string_format("Roll-at-Max-Delta=%f degrees", _result->roll_at_deltamax));
    _result->add_evidence(string_format("Desired-Roll-at-Max-Delta=%f degrees", _result->desroll_at_deltamax));
    _result->add_evidence(string_format("Pitch-at-Max-Delta=%f degrees", _result->pitch_at_deltamax));
    _result->add_evidence(string_format("Desired-Pitch-at-Max-Delta=%f degrees", _result->despitch_at_deltamax));

    // FIXME: subclass rather than this:
    if (_vehicle->vehicletype() == AnalyzerVehicle::Base::vehicletype_t::copter) {
        AnalyzerVehicle::Copter *&copter = (AnalyzerVehicle::Copter*&)_vehicle;
        std::set<uint8_t> high = _result->motors_clipping_high;
        std::set<uint8_t> low = _result->motors_clipping_low;
        std::set<uint8_t> failing = _result->motors_failing;
        for (uint8_t i=1; i <= copter->num_motors(); i++) {
            if (high.count(i)) {
                _result->add_evidence(string_format("Motor %d clipping high", i));
            } else if (low.count(i)) {
                _result->add_evidence(string_format("Motor %d clipping low", i));
            }
            if (failing.count(i)) {
                _result->add_evidence(string_format("Motor %d failed", i));
            }
        }
    }
    
    add_result(_result);
    _result = NULL;
}
void Analyzer_Attitude_Control::evaluate()
{
    if (!_vehicle->is_flying()) {
        return;
    }

    float roll = _vehicle->roll();
    float desroll = _vehicle->desroll();
    float roll_delta = fabs(roll - desroll);

    float pitch = _vehicle->pitch();
    float despitch = _vehicle->despitch();
    float pitch_delta = fabs(pitch - despitch);

    float delta = (pitch_delta > roll_delta) ? pitch_delta : roll_delta;

    // ::fprintf(stderr,"%lu attitude control roll=%f desroll=%f (delta=%f vs warn=%f)\n",_vehicle->T(), roll, desroll, delta, offset_warn);
    if (_result == NULL) {
        if (delta > offset_warn) {
            // start a new incident
            open_result(delta);
        }
    } else {
        // incident is currently underway
        if (delta < offset_warn) {
            close_result();
        } else {
            update_result(delta);
        }
    }
}

void Analyzer_Attitude_Control::end_of_log(uint32_t packet_count UNUSED)
{
    if (_result != NULL) {
        close_result();
    }

    if (_vehicle->roll_modtime() == 0) {
        Analyzer_Result_Summary *result = new Analyzer_Result_Summary();
        result->set_status(analyzer_status_warn);
        result->set_reason("Vehicle attitude never set");
        result->set_severity_score(5);
        result->add_source(_data_sources.get("ATTITUDE"));
        add_result(result);
    }
}
