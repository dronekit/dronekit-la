# DroneKit LA

[![Join the chat at https://gitter.im/dronekit/dronekit-la](https://badges.gitter.im/dronekit/dronekit-la.svg)](https://gitter.im/dronekit/dronekit-la?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

Log Analyzer for ArduPilot DataFlash logs and MAVLink telemetry logs

[![Circle CI](https://circleci.com/gh/dronekit/dronekit-la/tree/master.svg?style=svg)](https://circleci.com/gh/dronekit/dronekit-la/tree/master)


## Building

Requirements:
- `build-essential` package
- `g++` >= 4.8
- `libjsoncpp-dev` && `libjsoncpp0`

Use the `Makefile` in the root directory

```
make
```

## Adding new tests/analyzers
 - create a new header file for your test (e.g. analyzer_gyro_drift.h)
  - copy in the contents of another, similar header file
   - in this case, I chose analyzer_compass_vector_length.h as they're testing similar things and will both produce Analyer_Result_Period results
  - replace all instances of e.g. "compass_vector_length" with "gyro_drift"
  - trim out of the Result all variables and method definitions which don't look appropriate for your new test result
  - sometimes multiple different result objects are generated for the one Analyzer  Remove these if not needed
  - change the test name
  - change the test description
 
## Adding new data sources
There is a great deal of data in logs, and not all of it is currently made available for the analyzers to utilise.

The method to take data from a data source and make it available to the analyzers differs based on the data source:

 - if there is no place in the vehicle to store the data (see analyzervehicle.h), add an appropriate field
  - always bear in mind that this data may also come from some other place as well, so the abstraction should avoid coupling against the message type.

### DataFlash logs
 - subclassing is used to handle different message types
 - In LA_MsgHandler.h, create a new subclass for the message type you are interested in
 - in LA_MsgHandler.cpp, create your implementation, updating the vehicle model as appropriate.

### MAVLink (telemetry) logs
 - polymorphism is used to handle different message types
 - add declaration of new handle_decoded_message method to analyzing_mavlink_message_handler.h
 - add add_data_source lines to Analyzing_MAVlink_Message_Handler constructor in analyzing_mavlink_message_handler.cpp
 - add implementation of new handle_decoded_message to analyzing_mavlink_message_handler.cpp

## Analyzers to Write:
 - analyzer_default_params_used
   - fails if we ever need to use a default value (i.e. this log didn't give us parameters straight-up)
 - analyzer for time take to pass prearm checks
 - plane trims:
  - here's an example of bad trims:
RC1_MAX 1773.000000
RC1_MIN 1078.000000
RC1_REV -1.000000
RC1_TRIM 1527.000000
 (there's ~500 to the left, ~250 to the right - that's probably a very bad thing)

 - accelerometer and gyro drift
 - accelerometer and gyro disagreement
 - check for obviously bad parameters (e.g. ANGLE_MAX < 1000)

 - work out what we use RATE for

 - detect GPS resets by position fix going low for some period of time

## FIXMEs
 - the attitude control tests should ignore periods where the craft is not armed
 - the brownout report should give relative, not absolute, height at end of flight
 
    


// - two fundamental types of test
//  - is the software working correctly (EKF issues)
//  - is the vehicle doing sensible things (attitude etc)
