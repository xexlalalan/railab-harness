#!/usr/bin/env bash
# 10: platform down 1000 steps, dose 9.99 g of liquid by weight, platform up 1000 steps, open the housing
#     ./10_water_9_99g.sh [g] [steps]   firmware: + = up, - = down
source "$(dirname "$0")/lib.sh"
G=${1:-9.99}; STEPS=${2:-1000}
step bunny platform "-$STEPS"
step bunny liquid weigh "$G"
step bunny platform "$STEPS"
step bunny housing open
