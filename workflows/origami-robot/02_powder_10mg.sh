#!/usr/bin/env bash
# 2: close housing, tare, dispense 10 mg powder, open housing
source "$(dirname "$0")/lib.sh"
MG=${1:-10}
step bunny housing close
step bunny tare
step bunny weigh-to "$(awk "BEGIN{print $MG/1000}")" powder
step bunny housing open
