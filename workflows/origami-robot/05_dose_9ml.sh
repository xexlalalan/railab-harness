#!/usr/bin/env bash
# 5: liquid doser pushes 9 mL of water (one dosing per run). ./05_dose_9ml.sh [mL] [steps/s]
source "$(dirname "$0")/lib.sh"
ML=${1:-9}; RATE=${2:-1200}; UL_PER_STEP=0.556
step bunny liquid push "$(awk "BEGIN{printf \"%d\", $ML*1000/$UL_PER_STEP}")" "$RATE"
