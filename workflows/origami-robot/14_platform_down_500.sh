#!/usr/bin/env bash
# 14: Bunny platform down 500 steps. ./14_platform_down_500.sh [steps]   firmware: + = up, - = down
source "$(dirname "$0")/lib.sh"
step bunny platform "-${1:-500}"
