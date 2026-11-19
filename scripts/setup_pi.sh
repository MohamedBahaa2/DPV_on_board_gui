#!/bin/bash
# One-time Raspberry Pi setup for the DPV dashboard.
#   sudo /home/dpv/dpv_gui/scripts/setup_pi.sh
# The Pixhawk is on USB, so no UART/boot-config changes are needed.
set -euo pipefail
[ "$(id -u)" -eq 0 ] || { echo "Run with sudo."; exit 1; }

# Never blank the screen (it's a dashboard). Takes effect at next login/reboot.
raspi-config nonint do_blanking 1

# USB serial (/dev/ttyACM*) needs the dialout group.
id -nG dpv | grep -qw dialout || usermod -aG dialout dpv

echo "Done. Reboot to apply:  sudo reboot"
