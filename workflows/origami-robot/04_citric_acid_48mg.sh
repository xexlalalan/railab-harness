#!/usr/bin/env bash
# 4: tare, weigh 48 mg (0.048 g) of "Citric Acid" (powder looked up by name in the Bunny's database)
#    ./04_citric_acid_48mg.sh [g] [powder name or id]
source "$(dirname "$0")/lib.sh"
G=${1:-0.048}; POWDER=${2:-${POWDER:-Citric Acid}}
step bunny tare
step bunny weigh-to "$G" powder "$POWDER"
