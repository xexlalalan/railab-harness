#!/usr/bin/env bash
# 13: Bunny platform up 500 steps. ./13_platform_up_500.sh [steps]   firmware: + = up, - = down
source "$(dirname "$0")/lib.sh"
step bunny platform "${1:-500}"
