#! /bin/bash
espflash flash --chip esp32s3 --baud 1000000 "$1" && probe-rs attach --chip=esp32s3 --always-print-stacktrace --no-location "$1"
