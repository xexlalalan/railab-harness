#!/usr/bin/env bash
# 6: pipette aspirates 1 mL. ./06_pipette_aspirate_1ml.sh [uL]
source "$(dirname "$0")/lib.sh"
step pipette aspirate "${1:-1000}"
