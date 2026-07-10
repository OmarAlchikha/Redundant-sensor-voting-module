#!/usr/bin/env bash
# Build and run the hardware-free voting simulation.
set -euo pipefail
cd "$(dirname "$0")"
CXX="${CXX:-g++}"
"$CXX" -std=c++11 -Wall -Wextra -O2 -I../sensor_voting \
    sim_demo.cpp ../sensor_voting/voting_core.cpp -o sim_demo
./sim_demo
