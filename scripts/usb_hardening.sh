#!/usr/bin/env bash
#
# USB hardening for the arm RealSense + the shared PCH bus (lidars, hubs).
#
# Why: the arm-mounted RealSense D435i occasionally stalls its depth stream; the
# realsense2 nodelet's USBDEVFS_CLEAR_HALT recovery escalates to a controller-wide
# reset that drops the two lidars it shares the PCH xHCI controller with. This
# applies librealsense's recommended USB settings so the camera sustains its
# 1280x720@30 depth+color config, and narrows the reset blast radius:
#   1. usbfs_memory_mb -> 128   (kernel default 16 is too small for the dual stream)
#   2. USB autosuspend off for the RealSense, CP210x lidars, and Realtek hubs
# It also installs the persistent udev rule for (2).
#
# The ZED is on a separate (Thunderbolt) controller and is intentionally NOT
# touched by this script.
#
# Usage:  sudo scripts/usb_hardening.sh
#
# usbfs_memory_mb here is applied live only; persistence across reboots is a
# one-time GRUB change (printed at the end).

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RULE_SRC="${REPO_ROOT}/config/udev/99-usb-hardening.rules"
RULE_DST="/etc/udev/rules.d/99-usb-hardening.rules"
USBFS_MB=128

# Autosuspend-off targets: idVendor:idProduct of devices on the shared PCH bus.
TARGETS=("8086:0b3a" "10c4:ea60" "0bda:5411" "0bda:0411")

if [[ ${EUID} -ne 0 ]]; then
  echo "This script needs root. Re-run: sudo $0" >&2
  exit 1
fi

echo "[1/3] usbfs_memory_mb: $(cat /sys/module/usbcore/parameters/usbfs_memory_mb) -> ${USBFS_MB} (live)"
echo "${USBFS_MB}" > /sys/module/usbcore/parameters/usbfs_memory_mb

echo "[2/3] Disabling USB autosuspend on connected target devices (live)"
for dev in /sys/bus/usb/devices/*; do
  [[ -r "${dev}/idVendor" && -r "${dev}/idProduct" ]] || continue
  id="$(cat "${dev}/idVendor"):$(cat "${dev}/idProduct")"
  for t in "${TARGETS[@]}"; do
    if [[ "${id}" == "${t}" && -w "${dev}/power/control" ]]; then
      echo "on" > "${dev}/power/control"
      echo "    ${dev##*/}  ${id}  power/control=on"
    fi
  done
done

echo "[3/3] Installing udev rule -> ${RULE_DST}"
install -m 0644 "${RULE_SRC}" "${RULE_DST}"
udevadm control --reload-rules
udevadm trigger --subsystem-match=usb --action=add

cat <<'EOF'

Done (live settings + persistent udev rule installed).

To persist usbfs_memory_mb across reboots, add the kernel arg to GRUB once
(check it is not already present first):

  grep -q 'usbcore.usbfs_memory_mb' /etc/default/grub \
    || sudo sed -i 's/\(GRUB_CMDLINE_LINUX_DEFAULT="[^"]*\)"/\1 usbcore.usbfs_memory_mb=128"/' /etc/default/grub
  sudo update-grub
  # takes effect on next reboot

Verify:
  cat /sys/module/usbcore/parameters/usbfs_memory_mb        # -> 128
  for d in /sys/bus/usb/devices/*; do \
    [ -r "$d/idProduct" ] && [ "$(cat $d/idProduct)" = 0b3a ] && \
    echo "RealSense $d power/control=$(cat $d/power/control)"; done   # -> on
EOF
