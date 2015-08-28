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
