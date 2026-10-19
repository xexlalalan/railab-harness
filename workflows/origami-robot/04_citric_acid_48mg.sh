#!/usr/bin/env bash
# 4: tare, weigh 48 mg (0.048 g) of Citric acid (id 3123533989)
#    ./04_citric_acid_48mg.sh [g] [powder id]     POWDER=<barcode id> overrides the powder (default 3123533989 = Citric acid)
source "$(dirname "$0")/lib.sh"
G=${1:-0.048}; POWDER=${2:-${POWDER:-3123533989}}
step bunny tare
step bunny weigh-to "$G" powder "$POWDER"
