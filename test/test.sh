#!/bin/sh

ANALYZER=./dronekit-la
TLOG=test/logs/flight.tlog
TESTPY=test/test.py

set -e
set -x

$ANALYZER $TLOG >/dev/null
$ANALYZER -c /dev/null $TLOG >/dev/null
$ANALYZER -c /dev/null -s json $TLOG >/dev/null
$ANALYZER -h
$ANALYZER -s bad_output_format $TLOG
$ANALYZER logs/this-log-does-not-exist
$ANALYZER -s json $TLOG >/dev/null
$ANALYZER -c very_unlikely_to_exist json $TLOG >/dev/null && exit 1
$ANALYZER -s plain-text $TLOG >/dev/null
$ANALYZER -s brief $TLOG >/dev/null
$ANALYZER -s html $TLOG >/dev/null
$ANALYZER -a 'Good EKF' $TLOG >/dev/null

$TESTPY

exit 0
