#!/usr/bin/env bash
# 4: close the housing, platform down 1000 steps, tare, weigh 48 mg of Citric acid (id 3123533989), platform up 1000 steps, open the housing
#    ./04_citric_acid_48mg.sh [mg] [steps]     POWDER=<barcode id> overrides the powder (default 3123533989 = Citric acid)
#    firmware: platform + = up, - = down     (same sequence as self-demo-1 step 1)
source "$(dirname "$0")/lib.sh"
MG=${1:-48}; STEPS=${2:-1000}; POWDER=${POWDER:-3123533989}
step bunny housing close
step bunny platform "-$STEPS"
step bunny tare
step bunny weigh-to "$(awk "BEGIN{print $MG/1000}")" powder "$POWDER"
step bunny platform "$STEPS"
step bunny housing open
