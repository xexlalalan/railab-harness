#!/usr/bin/env bash
# 4: close the housing, tare, weigh 48 mg of Citric acid (id 3123533989), open the housing
#    ./04_citric_acid_48mg.sh [mg]     POWDER=<barcode id> overrides the powder (default 3123533989 = Citric acid)
source "$(dirname "$0")/lib.sh"
MG=${1:-48}; POWDER=${POWDER:-3123533989}
step bunny housing close
step bunny tare
step bunny weigh-to "$(awk "BEGIN{print $MG/1000}")" powder "$POWDER"
step bunny housing open
