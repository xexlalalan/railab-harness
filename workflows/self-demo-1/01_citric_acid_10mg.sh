#!/usr/bin/env bash
# 1: close the housing, platform down 1000 steps, tare, weigh 10 mg of "Citric Acid", platform up 1000 steps, open the housing
#    ./01_citric_acid_10mg.sh [mg] [steps]     POWDER="Citric Acid" (a name: the Bunny resolves it in its database, newest entry) or a barcode id
#    firmware: platform + = up, - = down
source "$(dirname "$0")/lib.sh"
MG=${1:-10}; STEPS=${2:-1000}; POWDER=${POWDER:-Citric Acid}
step bunny housing close
step bunny platform "-$STEPS"
step bunny tare
step bunny weigh-to "$(awk "BEGIN{print $MG/1000}")" powder "$POWDER"
step bunny platform "$STEPS"
step bunny housing open
