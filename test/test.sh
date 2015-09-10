#!/bin/sh

ANALYZER=../loganalyzer

set -e
set -x

$ANALYZER logs/1.solo.tlog >/dev/null
$ANALYZER -c /dev/null logs/1.solo.tlog >/dev/null
$ANALYZER -c /dev/null -s json logs/1.solo.tlog >/dev/null
$ANALYZER -h
$ANALYZER -s bad_output_format logs/1.solo.tlog
$ANALYZER logs/this-log-does-not-exist
$ANALYZER -s json logs/1.solo.tlog >/dev/null
$ANALYZER -c very_unlikely_to_exist json logs/1.solo.tlog >/dev/null

