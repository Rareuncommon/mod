#!/usr/bin/env bash
# Builds and runs the prediction tests (tests/test_*.cpp), or the ones named:
#   tests/run.sh                 all tests
#   tests/run.sh test_wave       one test
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p build
status=0
if [ $# -gt 0 ]; then names=("$@"); else names=(); for f in test_*.cpp; do [ -e "$f" ] && names+=("${f%.cpp}"); done; fi
for name in "${names[@]}"; do
    name="${name%.cpp}"
    if ! g++ -std=c++20 -O1 -g -Wall -Wextra -fsanitize=address,undefined -o "build/$name" "$name.cpp"; then
        echo "$name: build failed"; status=1; continue
    fi
    if ! "./build/$name"; then status=1; fi
done
exit $status
