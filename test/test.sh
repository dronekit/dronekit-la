#!/bin/sh

ANALYZER=../dataflash_logger

set -e
set -x

$ANALYZER -c /dev/null logs/1.solo.tlog >/dev/null
$ANALYZER -c /dev/null -s json logs/1.solo.tlog >/dev/null
$ANALYZER -h
$ANALYZER -s json logs/1.solo.tlog >/dev/null
$ANALYZER -c very_unlikely_to_exist json logs/1.solo.tlog >/dev/null

