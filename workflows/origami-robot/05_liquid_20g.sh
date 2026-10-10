#!/usr/bin/env bash
# 5: tare, dose 20 g of liquid by weight. ./05_liquid_20g.sh [g]
source "$(dirname "$0")/lib.sh"
G=${1:-20}
step bunny tare
step bunny liquid weigh "$G"
