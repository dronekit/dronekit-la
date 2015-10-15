# DroneKit LA

Log Analzer for ArduPilot

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
Ensure the paket types you are interested in are being handled in:
 - mavlink_message_handler.h
 - ensure mavlink_reader.cpp mentions your packet type in handle_message_received
 - ensure analyze.h mentions the appropriately-signaured functions


Analyzers to Write:
 - analyzer_default_params_used
   - fails if we ever need to use a default value
 - analyzer for time take to pass prearm checks
 - BARO drift
   - fails if any two BAROs drift from each other
 - vehicle detected
   - fails hard if we never detect a vehicle type

 - plane trims:
  - here's an example of bad trims:
RC1_MAX 1773.000000
RC1_MIN 1078.000000
RC1_REV -1.000000
RC1_TRIM 1527.000000
 (there's ~500 to the left, ~250 to the right - that's probably a very bad thing)



*NOTE*
 - why are tlogs coming in with no parameters?!
 - *especially* why are tlogs coming in with no messages indicating what the UAV iS?
