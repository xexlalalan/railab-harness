#!/usr/bin/env bash
# 10: dose 9.99 g of liquid by weight, then open the housing. ./10_water_9_99g.sh [g]
source "$(dirname "$0")/lib.sh"
step bunny liquid weigh "${1:-9.99}"
step bunny housing open
