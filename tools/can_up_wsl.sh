#!/usr/bin/env bash
# Bring the candleLight USB-CAN adapter (PIPER) up inside WSL2 at 1 Mbps.
# Run from the WSL shell as the normal user; root steps go through `wsl.exe -u root`
# because interactive sudo is not available in automated sessions.
# Prereq (once per Windows boot, needs one UAC approval):  usbipd bind --busid <BUSID>
set -euo pipefail
IFACE="${1:-can0}"
BITRATE="${2:-1000000}"

BUSID=$(usbipd.exe list 2>/dev/null | tr -d '\r' | awk '/1d50:606f/ && $1 ~ /^[0-9]+-[0-9]+$/ {print $1; exit}')
if [[ -z "${BUSID}" ]]; then
  echo "no candleLight adapter (1d50:606f) visible to usbipd -- is it plugged in?" >&2; exit 1
fi
if ! lsusb 2>/dev/null | grep -q 1d50:606f; then
  echo "attaching busid ${BUSID} to WSL"
  usbipd.exe attach --wsl --busid "${BUSID}"
  sleep 2
fi
wsl.exe -d "${WSL_DISTRO_NAME}" -u root -- sh -c "
  modprobe gs_usb && modprobe can_raw &&
  ip link set ${IFACE} down 2>/dev/null;
  ip link set ${IFACE} type can bitrate ${BITRATE} && ip link set ${IFACE} up" | tr -d '\0'
ip -details link show "${IFACE}" | grep -E "state|bitrate"
echo "sanity: expect a stream of 0x2A1..0x2A8 / 0x251..0x256 frames"
timeout 1 candump "${IFACE}" | head -5 || true
