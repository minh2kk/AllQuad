#!/bin/sh
# Build and run every simulator scenario (needs g++). Single scenario: tools/run_sim.sh mission
set -e
cd "$(dirname "$0")/.."
tools/build_host.sh
./build/sim "${1:-all}"
