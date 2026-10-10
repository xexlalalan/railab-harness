#!/usr/bin/env bash
# 8: pipette aspirates 1 mL at 500 uL/s with 1 uL pre-aspirated air. ./08_pipette_aspirate_1ml.sh [uL] [uL/s] [pre uL]
source "$(dirname "$0")/lib.sh"
step pipette aspirate "${1:-1000}" "${2:-500}" "${3:-1}"
