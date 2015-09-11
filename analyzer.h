#ifndef ANALYZER_H
#define ANALYZER_H

#include <jsoncpp/json/json.h> // libjsoncpp0 and libjsoncpp-dev on debian
#include <jsoncpp/json/writer.h> // libjsoncpp0 and libjsoncpp-dev on debian

#include "analyzervehicle.h"
#include "INIReader.h"

enum analyzer_status {
    analyzer_status_warn = 17,
    analyzer_status_fail,
    analyzer_status_ok,
};

class analyzer_result {
public:

    const char *status_as_string() const {
        switch(_status) {
        case analyzer_status_fail:
            return "FAIL";
        case analyzer_status_warn:
            return "WARN";
        case analyzer_status_ok:
            return "OK";
        }
        return "STRANGE";
    }
    analyzer_status status() { return _status; }
    void set_status(analyzer_status status) { _status = status; }

    void set_reason(const std::string reason) {
        if (_reason != NULL) {
            delete(_reason);
        }
        _reason = new std::string(reason);
    }
    const std::string *reason() const { return _reason; }

    virtual void to_json(Json::Value &root) const;

    void add_evidence(const std::string f) {
        _evidence.push_back(f);
    }
    void add_series(const std::string f) {
        _series.push_back(f);
    }

    void add_evilness(uint32_t evilness) {
        _evilness += evilness;
    }
    uint32_t evilness() const {
        return _evilness;
    }

private:
    analyzer_status _status = analyzer_status_ok;
    std::string *_reason = NULL;

    std::vector<std::string> _evidence;
    std::vector<std::string> _series;

    void to_json_add_array(Json::Value &root,
                           std::string name,
                           std::vector<std::string> array) const;
    uint32_t _evilness = 0;
};

class Analyzer_Result_Period : public analyzer_result {
public:
    Analyzer_Result_Period() :
        analyzer_result()
        { }

    virtual void to_json(Json::Value &root) const override;
    
    // FIXME: scope
    uint64_t _T_start = 0;
    uint64_t _T_stop;

private:
};

class Analyzer_Result_Summary : public analyzer_result {
public:
    Analyzer_Result_Summary() :
        analyzer_result()
        { }

    // virtual void to_json(Json::Value &root) const override;

private:
};


class Analyzer_Result_Event : public analyzer_result {
public:
    Analyzer_Result_Event() :
        analyzer_result()
        { }

    virtual void to_json(Json::Value &root) const;
    
    // FIXME: scope
    uint64_t _T;

private:
};


class Analyzer {

public:
    Analyzer(AnalyzerVehicle::Base *&vehicle) :
        _vehicle(vehicle)
        { }

    virtual bool configure(INIReader *config) {
        return true;
    }

    virtual const char *name() const = 0;
    virtual const char *description() const = 0;
    virtual void results_json_results(Json::Value &root);
    virtual void end_of_log(uint32_t packet_count) { }


    std::vector<analyzer_result*> results() {
        return _results;
    }

    void add_severity_score(uint8_t sin_points) {
        _severity_score += sin_points;
    }
    virtual uint16_t get_severity_score() const { return _severity_score; }

    analyzer_status status() const { return _status; }

    virtual void evaluate() { }

    virtual void add_result(analyzer_result* result) {
        _results.push_back(result);
    }

protected:
    std::string to_string(double x);

    AnalyzerVehicle::Base *&_vehicle;

    uint16_t _severity_score = 0;
    void set_status(analyzer_status status) { _status = status; }

    // FIXME: scope
    std::vector<analyzer_result*> _results;

private:
    analyzer_status _status = analyzer_status_ok;
};

#endif
    


// - two fundamental types of test
//  - is the software working correctly (EKF issues)
//  - is the vehicle doing sensible things (attitude etc)
