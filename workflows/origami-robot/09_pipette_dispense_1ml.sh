#!/usr/bin/env bash
# 9: pipette dispenses 1 mL at 500 uL/s with 100 uL blow-out after. ./09_pipette_dispense_1ml.sh [uL] [uL/s] [post uL]
source "$(dirname "$0")/lib.sh"
step pipette dispense "${1:-1000}" "${2:-500}" "${3:-100}"
