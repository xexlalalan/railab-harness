#!/usr/bin/env bash
# 5: platform down 100 steps, wait 3 s, dispense powder to 10 mg, platform up 100 steps
#    ./05_powder_10mg.sh [mg] [steps]   PL_DOWN=-1 flips the platform sign if "down" moves the wrong way
source "$(dirname "$0")/lib.sh"
MG=${1:-10}; STEPS=${2:-100}; PL_DOWN=${PL_DOWN:-1}
step bunny platform $(( PL_DOWN * STEPS ))
sleep 3
step bunny weigh-to "$(awk "BEGIN{print $MG/1000}")" powder
step bunny platform $(( -PL_DOWN * STEPS ))
