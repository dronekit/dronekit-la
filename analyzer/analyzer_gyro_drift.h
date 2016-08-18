#ifndef ANALYZE_GYRO_DRIFT_H
#define ANALYZE_GYRO_DRIFT_H

/*
 * analyze_attitudecontrol
 *
 */

#include "analyzer.h"

class Analyzer_Gyro_Drift_Result : public Analyzer_Result_Period {
    friend class Analyzer_Gyro_Drift;
public:
    Analyzer_Gyro_Drift_Result(std::string name) :
        _name(name) { }
    const std::string name() const { return _name; };
    void to_json(Json::Value &root) const;


private:
    const std::string _name;
    double _max_delta = 0;
    uint8_t _axis;
};


class Analyzer_Gyro_Drift : public Analyzer {

public:

    Analyzer_Gyro_Drift(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
	Analyzer(vehicle,data_sources)
    { }

    const std::string name() const override { return "Gyro Drift"; }
    const std::string description() const override {
        return "Gyroscopes sometimes start to register movement where there is none.  This test will FAIL or WARN if the any gyroscope's average acceleration on any axis begins to drift.";
    }

    bool configure(INIReader *config) override;

    void evaluate_gyro(const AnalyzerVehicle::IMU *first,
                       const AnalyzerVehicle::IMU *imu);
    void evaluate() override;

    void for_each_gyro(const std::function <void (AnalyzerVehicle::Compass*)> f);

private:
    std::map<const std::string, Analyzer_Gyro_Drift_Result *> _result;

    float _delta_warn = 0.09f;
    float _delta_fail = 0.1f;

    uint64_t _gyr_avg_usec = 2000000;
    uint64_t _duration_min = 500000; // microseconds

    void end_of_log(uint32_t packet_count) override;

    void open_result(const std::string result_key,
                     const AnalyzerVehicle::IMU *first,
                     const AnalyzerVehicle::IMU *imu,
                     const uint8_t axis,
                     const double delta);
    void update_result(Analyzer_Gyro_Drift_Result *result, const Vector3f acc_delta);
    void update_result(Analyzer_Gyro_Drift_Result *, double length);
    void close_result(const std::string result_key, Analyzer_Gyro_Drift_Result *);

    void close_results();
};

#endif
