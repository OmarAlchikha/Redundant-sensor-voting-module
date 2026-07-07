#!/usr/bin/env bash
# Build and run the host-side voting-core unit tests. No Arduino toolchain
# required -- voting_core.cpp is plain C++11.
set -euo pipefail
cd "$(dirname "$0")"

CXX="${CXX:-g++}"
"$CXX" -std=c++11 -Wall -Wextra -Werror -O2 \
    -I../sensor_voting \
    test_voting.cpp ../sensor_voting/voting_core.cpp \
    -o run_tests

./run_tests
