#ifndef ANALYZER_SUBSYS_ERRORS_H
#define ANALYZER_SUBSYS_ERRORS_H

/*
 * analyzer_subsys_errors
 *
 */

#include "analyzer.h"
#include "data_sources.h"


class Analyzer_Subsys_Errors_Result : public Analyzer_Result_Period {
public:
    Analyzer_Subsys_Errors_Result(uint32_t subsys,
                                  const std::string subsys_string,
                                  uint32_t ecode,
                                  const std::string ecode_string) :
        _subsys(subsys),
        _subsys_string(subsys_string),
        _ecode(ecode),
        _ecode_string(ecode_string)
        { }
    void to_json(Json::Value &root) const;
    uint32_t ecode() const { return _ecode; }

    std::string ecode_string() const { return _ecode_string; }
    std::string subsys_string() const { return _subsys_string; }

private:
    const uint32_t _subsys;
    const std::string _subsys_string;
    const uint32_t _ecode;
    const std::string _ecode_string;

};

class Analyzer_Subsys_Errors : public Analyzer {

public:
    Analyzer_Subsys_Errors(AnalyzerVehicle::Base *&vehicle,
                         Data_Sources &data_sources) :
    Analyzer(vehicle, data_sources)
    { }


    const std::string name() const override { return "Error Flags"; }
    const std::string description() const override {
        return "A UAV can have ongoing errors which it is managing.  This test will fail if such errors occur";
    }

    bool configure(INIReader *config) override;

private:

    std::map<uint32_t, uint32_t> _subsys_errors = { };
    std::map<uint32_t, Analyzer_Subsys_Errors_Result*> _results = { };

    void evaluate() override;

    void close_result(const uint32_t code);
    void open_result(const uint32_t code, const uint32_t ecode);

    void end_of_log(uint32_t packet_count) override;

    const std::string subsys_string(const uint32_t subsys) const;
    const std::string ecode_string(const uint32_t subsys,
                                   const uint32_t ecode) const;
};

#endif


