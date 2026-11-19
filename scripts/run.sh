#!/bin/bash
# Launch the DPV dashboard and relaunch it if it ever exits/crashes.
# Started at login by ~/.config/autostart/dpv-dash.desktop (see deploy/).
# Extra arguments are passed through, e.g.  scripts/run.sh --sim
#
# Stop it:     pkill -f scripts/run.sh; pkill -f dpv_dash
# Disable it:  touch ~/.config/dpv_dash.disabled   (remove the file to re-enable)

APP_DIR="$(cd "$(dirname "$0")/.." && pwd)"
LOG_DIR="$HOME/.local/state"
LOG="$LOG_DIR/dpv_dash.log"
mkdir -p "$LOG_DIR"

[ -e "$HOME/.config/dpv_dash.disabled" ] && exit 0

# Only one launcher at a time.
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/dpv_dash.lock"
flock -n 9 || { echo "dpv_dash already running"; exit 0; }

export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-wayland}"
cd "$APP_DIR" || exit 1

while true; do
    # keep the log from growing without bound
    if [ -f "$LOG" ] && [ "$(stat -c %s "$LOG")" -gt 5000000 ]; then
        mv -f "$LOG" "$LOG.1"
    fi
    echo "=== start $(date -Is) $*" >> "$LOG"
    "$APP_DIR/.venv/bin/python" -m dpv_dash "$@" >> "$LOG" 2>&1
    echo "=== exit code $? $(date -Is)" >> "$LOG"
    [ -e "$HOME/.config/dpv_dash.disabled" ] && exit 0
    sleep 2
done
