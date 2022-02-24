#ifndef ANALYZER_MOTORBALANCE_H
#define ANALYZER_MOTORBALANCE_H

/*
 * analyzer_motorbalance
 *
 */

#include "analyzer.h"
#include "data_sources.h"


class Analyzer_MotorBalance_Result_Axis : public Analyzer_Result_Summary {
public:
    Analyzer_MotorBalance_Result_Axis()
        { }

    void set_delta(const uint16_t delta) { _delta = delta; }
    uint16_t delta() { return _delta; }
    void set_axis(const char *axis) { memcpy(_axis,axis, strlen(axis)); }
    char *axis() { return _axis; }

private:
    uint16_t _delta;
    char _axis[5] = {};
};

class Analyzer_MotorBalance_Result_Motor : public Analyzer_Result_Summary {
public:
    Analyzer_MotorBalance_Result_Motor()
        { }

private:

};

class Analyzer_MotorBalance : public Analyzer {

public:
    Analyzer_MotorBalance(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
    Analyzer(vehicle, data_sources)
    { }

    const std::string name() const override { return "Motor Balance"; }
    const std::string description() const override {
        return "Detects copter balance and twist issues";
    }

private:

    const uint32_t _stable_min_duration = 100000; // 100ms
    const float _stable_pitch_rate_max = 1.0; // degrees/second
    const float _stable_roll_rate_max = 1.0; // degrees/second
    const float _stable_yaw_rate_max = 1.0; // degrees/second
    bool stable();

    // FIXME: change to percents, but quadplane doesn't currently seem to
    // use RCx_MIN and RCx_MAX to constrain its output
    uint16_t _attitude_pwm_delta_warn = 50;
    uint16_t _attitude_pwm_delta_fail = 100;
    uint16_t _yaw_pwm_delta_warn = 50;
    uint16_t _yaw_pwm_delta_fail = 100;
    uint16_t _motor_imbalance_warn = 50;
    uint16_t _motor_imbalance_fail = 100;

    uint64_t _stable_start_T = 0;
    uint64_t _total_pwm[16] = {};
    uint32_t _sample_count = 0;

    uint64_t _same_start_time = 0;
    const uint16_t _same_notflying_time = 10000; // 10 milliseconds
    bool _old_stabilizing = false;
    bool stabilizing();
    bool all_outputs_same() const;

    void evaluate_motors();

    void evaluate_copter();
    void evaluate_plane();
    void evaluate() override;

    void add_axis_result(const char *axis, const uint16_t imbalance, const uint16_t imbalance_warn, const uint16_t imbalance_fail);
    void add_motor_result(const uint8_t motornum, uint16_t motorsaverage, uint16_t motoraverage);

    void end_of_log_motors_x(const uint16_t * const averages);
    void end_of_log_motors();

    void end_of_log_copter();
    void end_of_log_plane();
    void end_of_log(uint32_t packet_count) override;

    uint16_t _motor_output_map[17] = {};
    bool _done_motor_output_map = false;

    uint8_t motorcount_for_frame_class(uint8_t value);
    uint8_t _motorcount;

    // from quadplane.cpp:
    typedef enum {
        frame_class_quad = 0,
        frame_class_hexa = 1,
        frame_class_octa = 2,
        frame_class_octaquad = 3,
        frame_class_y6 = 4,
    } frame_class_t;
    frame_class_t _frame_class;

    // from quadplane.cpp:
    typedef enum {
        frame_type_plus = 0,
        frame_type_x = 1,
        frame_type_v = 2,
        frame_type_h = 3,
        frame_type_vtail = 4,
        frame_type_atail = 5,
        frame_type_y6b = 10,
    } frame_type_t;
    frame_type_t _frame_type;
};

#endif


