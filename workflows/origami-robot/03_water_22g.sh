#!/usr/bin/env bash
# 3: close housing, tare, add 22 g water by weight, open housing
source "$(dirname "$0")/lib.sh"
G=${1:-22}
step bunny housing close
step bunny tare
step bunny liquid weigh "$G"
step bunny housing open
