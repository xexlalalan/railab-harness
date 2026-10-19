#!/usr/bin/env bash
# 5: half-close the housing, tare, dose 20 g of liquid by weight, open the housing
#    ./05_liquid_20g.sh [g]     (same sequence as self-demo-1 step 2)
source "$(dirname "$0")/lib.sh"
G=${1:-20}
step bunny housing half
step bunny tare
step bunny liquid weigh "$G"
step bunny housing open
