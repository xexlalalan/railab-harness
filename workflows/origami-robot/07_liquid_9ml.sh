#!/usr/bin/env bash
# 7: dose 9 mL of liquid by volume (open-loop pump steps from the Bunny's liquid profile, no scale). ./07_liquid_9ml.sh [mL]
source "$(dirname "$0")/lib.sh"
step bunny liquid volume "${1:-9}"
