#!/usr/bin/env bash
# 4: tare, weigh 4.803 g of "Citric Acid" (powder looked up by name in the Bunny's database)
#    ./04_citric_acid_4_803g.sh [g] [powder name or id]
source "$(dirname "$0")/lib.sh"
G=${1:-4.803}; POWDER=${2:-${POWDER:-Citric Acid}}
step bunny tare
step bunny weigh-to "$G" powder "$POWDER"
