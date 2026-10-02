#!/usr/bin/env bash
# 12: platform down 1000 steps. ./12_platform_down.sh [steps]
source "$(dirname "$0")/lib.sh"
step bunny platform "-${1:-1000}"
