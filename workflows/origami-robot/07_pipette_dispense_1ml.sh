#!/usr/bin/env bash
# 7: pipette dispenses 1 mL. ./07_pipette_dispense_1ml.sh [uL]
source "$(dirname "$0")/lib.sh"
step pipette dispense "${1:-1000}"
