# DroneKit LA

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

## Adding new tests/analyzers (FIXME)

Duplicate a test, change the obvious stuff
 - ensure the packet types you are interested in are being handled in mavlink_message_handler and LA_MsgHandler
 - ensure mavlink_reader.cpp mentions your packet type in handle_message_received
 - ensure analyze.h mentions the appropriately-signatured functions


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


## FIXMEs
 - the attitude control tests should ignore periods where the craft is not armed
 - the brownout report should give relative, not absolute, height at end of flight
 
