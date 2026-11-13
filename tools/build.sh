#!/bin/bash
# Build the read-only flange pose tool against the DRFL bundled in doosan-robot2 (needs libpoco-dev).
set -e
WS="$(cd "$(dirname "$0")/.." && pwd)"
LIB="$WS/src/doosan-robot2/dsr_common2/lib/humble/x86_64/libDRFL.a"
INC="$WS/src/doosan-robot2/dsr_common2/include"
g++ -std=c++17 -O2 -I"$INC" "$WS/tools/flange_pose.cpp" -o "$WS/tools/flange_pose" "$LIB" -lPocoFoundation -lPocoNet -lpthread -ldl
echo "built tools/flange_pose"
