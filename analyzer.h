/**
 * @file
 * @author Peter Barker <peter.barker@3drobotics.com>
 *
 * @section DESCRIPTION
 *
 * Base class for all analyzers; provides facilities to register results
 */

#ifndef ANALYZER_H
#define ANALYZER_H

#include <jsoncpp/json/json.h> // libjsoncpp0 and libjsoncpp-dev on debian
// #include <jsoncpp/json/writer.h> // libjsoncpp0 and libjsoncpp-dev on debian

#include "analyzervehicle.h"
#include "INIReader.h"

#include "data_sources.h"

#include "analyzer_util.h"

/// Enumeration of possible states for both an Analyzer_Result and Analyzer.
enum analyzer_status {
    analyzer_status_warn = 17,
    analyzer_status_fail,
    analyzer_status_ok,
};


/*!
 * Returns a textual interpretation of the supplied status.
 *
 * @param status Analyzer status to provide text for.
 * @return Textual interpretation of status.
 */
const char *_status_as_string(analyzer_status status);



/// @brief Base class for analyzer results; extend this to provide a custom result object.
///
/// You probably do not want to directly extend Analyzer_Result, but one of its immediate subclasses.
class Analyzer_Result {
public:
    /// @brief Construct an Analyzer_Result.
    virtual ~Analyzer_Result() { }

    /// @brief Provides a textual description of the Analyzer Result's status.
    /// @return A text string e.g. "OK" or "FAIL".
    const char *status_as_string() const {
        return _status_as_string(_status);
    }

    /// @brief Provide the Analyzer_Result's status.
    /// @return The Analyzer_Result's status.
    analyzer_status status() { return _status; }

    /// @brief Set the Analyzer Result's status.
    /// @param status The new status for this Result.
    void set_status(analyzer_status status) { _status = status; }

    /// @brief Set a simple, human-readable explanation of the result.
    /// @param reason The reason for the existence of this result.
    void set_reason(const std::string reason) {
        if (_reason != NULL) {
            delete(_reason);
        }
        _reason = new std::string(reason);
    }
    /// @brief Provide a simple, human-readable explanation if the result.
    const std::string *reason() const { return _reason; }

    /// @brief Provide analyzer output in the provided Json::Value.
    /// @param[out] Root object to populate with output.
    virtual void to_json(Json::Value &root) const;

    /// @brief Provide textual free-form evidence for the "reason".
    /// @param f Textual free-form evidence.
    void add_evidence(const std::string f) {
        _evidence.push_back(f);
    }

    /// @brief Indicate that a particular data source was used to come to the conclusion returned by reason().
    /// @param f A Data_Source relevant to the conclusion in reason().
    void add_source(const Data_Source *f) {
        if (f == NULL) {
            abort();
        }
        _sources.push_back(f);
    }

    /// @brief Indicate the result is incrementally more significant.
    /// @param evilness Degree to which this result is more significant.
    void increase_severity_score(uint32_t evilness) {
        _evilness += evilness;
    }
    /// @brief Indicate how significant this result is.
    /// @param evilness Degree of significance of this result.
    void set_severity_score(uint32_t evilness) {
        _evilness = evilness;
    }
    /// @brief Return a number indicating the relative significance of this result.
    /// @return The relative significance of this result.
    uint32_t severity_score() const {
        return _evilness;
    }

    void set_pure_output(bool pure) {
        _pure = pure;
    }
    bool pure_output() const {
        return _pure;
    }

private:
    analyzer_status _status = analyzer_status_ok;
    std::string *_reason = NULL;

    std::vector<std::string> _evidence;
    std::vector<const Data_Source*> _sources;

    void to_json_add_array(Json::Value &root,
                           std::string name,
                           std::vector<std::string> array) const;
    uint32_t _evilness = 0;

    bool _pure = false;
};


/// @brief Base class for an Analyzer Result which spans a period.
///
/// Examples would include a momentary attitude control loss.
class Analyzer_Result_Period : public Analyzer_Result {
public:
    Analyzer_Result_Period() :
        Analyzer_Result()
        { }

    virtual void to_json(Json::Value &root) const override;

    void set_T_start(const uint64_t start) { _T_start = start; }
    uint64_t T_start() const { return _T_start; }

    void set_T_stop(const uint64_t stop) { _T_stop = stop; }
    uint64_t T_stop() const { return _T_stop; }

    uint64_t duration() const { return _T_stop - _T_start; }

private:
    uint64_t _T_start = 0;
    uint64_t _T_stop;
};

/// @brief Base class for an Analyzer Result which provides information derived over the entire period of data input.
///
/// Examples would include "How many bytes of the input were processed".
class Analyzer_Result_Summary : public Analyzer_Result {
public:
    Analyzer_Result_Summary() :
        Analyzer_Result()
        { }

    // virtual void to_json(Json::Value &root) const override;

private:
};


/// @brief Base class for an Analyzer Result which does not span any time.
///
/// Examples would include a "Crash" event present in a log.
class Analyzer_Result_Event : public Analyzer_Result {
public:
    Analyzer_Result_Event() :
        Analyzer_Result()
        { }

    virtual void to_json(Json::Value &root) const;
    
    void set_T(uint64_t timestamp) {
        _T = timestamp;
    }

    uint64_t T() const {
        return _T;
    }

private:
    uint64_t _T;
};


/// @brief Base class for all analyzers.
///
/// @description Extend this class to create a new analyzer.
///
/// An Analyzer tracks changes to a vehicle model over time to
/// determine if anything untoward is happening.  "evaluate()" is
/// called repeatedly when the model's state changes, and "end_of_log"
/// is called when no more input will be forthcoming.  An Analyzer is
/// expected to output Analyzer_Result objects through the "add_result"
/// object.
class Analyzer {

public:
    /// @brief Constructor for Analyzer.
    /// @param vehicle The vehicle model to be analyzed.
    /// @param data_sources A mapping of abstract data source names to their concrete data sources (e.g. ATTITUDE -> [ ATT.Pitch, ATT.Roll ]).
    Analyzer(AnalyzerVehicle::Base *&vehicle, Data_Sources &data_sources) :
        _vehicle(vehicle),
        _data_sources(data_sources)
        { }

    virtual ~Analyzer() { }

    /// @brief Configure an analyzer from a .ini config file
    /// @param config The configuration source.
    /// @return true if configuration succeeded.
    virtual bool configure(INIReader *config UNUSED) {
        return true;
    }

    /// @brief a simple name that used to refer to this Analyzer.
    virtual const std::string name() const = 0;
    /// @brief Outlines the reason this test exists.
    virtual const std::string description() const = 0;

    /// @brief Provide analyzer output in the provided Json::Value.
    /// @param[out] root Object to populate with output
    virtual void results_json_results(Json::Value &root) const;

    /// @brief Return the analyzer's status represented as a string.
    const char *status_as_string() const {
        return _status_as_string(status());
    }

    /// @brief Set whether to produce output compatible with older versions.
    /// @param purity True if compatability fields should not be produced
    void set_pure_output(bool pure) {
        _pure = pure;
    }

    /// @brief Returns true if compatability fields will not be produced.
    /// @return true if compatability fields will not be produced.
    bool pure_output() const {
        return _pure;
    }

    /// @brief Return all results this analyzer has produced.
    std::vector<Analyzer_Result*> results() const {
        return _results;
    }
    /// @brief Return the number of results this analyzer has produced.
    uint16_t result_count() const {
        return _results.size();
    }

    /// @brief Returns a number "score" indicating how significant this result may be.
    virtual uint32_t severity_score() const;

    /// @brief Called whenever the vehicle's state changes.
    virtual void evaluate() = 0;

    /// @brief Called when no more input will be forthcoming.
    /// @param packet_count The number of packets processed to update the model.
    virtual void end_of_log(uint32_t packet_count UNUSED) { }

    /// @brief Provide the Analyzer's overall status.
    /// @return The Analyzer overall status.
    analyzer_status status() const;

protected:
    // @brief Add a result for this Analyzer.
    // @param result The result to add.
    virtual void add_result(Analyzer_Result* result) {
        result->set_pure_output(pure_output());
        _results.push_back(result);
    }

    /// @brief Provide the Analyzer Result's status.
    /// @return The Analyzer_Result's status.
    std::string to_string(double x);

    /// @brief Vehicle model.
    /// @details Updated by LA_MsgHandler* and analyzing_mavlink_message_handler, analyzed by analyzer_*.
    AnalyzerVehicle::Base *&_vehicle;
    /// @brief Map from abstract data source to concrete data source.
    /// @details e.g. BATTERY_REMAINING -> SYS_STATUS.battery_remaining.
    Data_Sources &_data_sources;

    bool _pure = false;

private:
    std::vector<Analyzer_Result*> _results;

};

#endif
