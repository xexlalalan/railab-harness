#!/usr/bin/env bash
# 2: half-close the housing, tare, dose 10 g of liquid by weight, open the housing
#    ./02_liquid_10g.sh [g]
source "$(dirname "$0")/lib.sh"
G=${1:-10}
step bunny housing half
step bunny tare
step bunny liquid weigh "$G"
step bunny housing open
