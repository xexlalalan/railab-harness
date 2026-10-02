#!/usr/bin/env bash
# 11: platform up 1000 steps. ./11_platform_up.sh [steps]
source "$(dirname "$0")/lib.sh"
step bunny platform "${1:-1000}"
