#!/usr/bin/env bash
# Native-gcc unit test for the movement estimator core. Usage: bash run_motion_test.sh
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
gcc -O2 -Wall -Wextra -I"$HERE/../include" \
    "$HERE/test_motion_estimator.c" "$HERE/../src-core0/motion_estimator.c" \
    -lm -o /tmp/test_motion_estimator || exit 1
/tmp/test_motion_estimator
