#!/usr/bin/env bash
# 1: close the housing, platform down 1000 steps, tare, weigh 10 mg of Citric acid (id 3123533989), platform up 1000 steps, open the housing
#    ./01_citric_acid_10mg.sh [mg] [steps]     POWDER=<barcode id> overrides the powder (default 3123533989 = newest Citric acid)
#    firmware: platform + = up, - = down
source "$(dirname "$0")/lib.sh"
MG=${1:-10}; STEPS=${2:-1000}; POWDER=${POWDER:-3123533989}
step bunny housing close
step bunny platform "-$STEPS"
step bunny tare
step bunny weigh-to "$(awk "BEGIN{print $MG/1000}")" powder "$POWDER"
step bunny platform "$STEPS"
step bunny housing open
