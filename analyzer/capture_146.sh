#!/usr/bin/env bash
# 13 s Airspy HF+ noise capture at 146 MHz, same settings as the controller's raw capture.
cd "$(dirname "$0")/.."
./build/airspyhf_zeromq/tools/src/airspyhf_zeromq_rx \
    -r ~/Downloads/airspy-hf.1.dat -f 146 -a 768000 -g off -m on -n 9984000
