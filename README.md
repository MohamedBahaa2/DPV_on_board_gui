# DPV Driver HUD

A heads-up-display style dashboard for the DPV driver. It runs on a Raspberry Pi 5 with a 5" HDMI screen. At power-on the Pi boots straight into a full-screen dashboard that shows speed, battery, driving mode and heading guidance read from the Pixhawk autopilot (ArduPilot Rover/Boat). The driver controls it with hall-effect push buttons. There is no touch input.

![layout](docs/hud.png)

---

## 1. How it fits together

```
 ┌──────────────┐  USB (MAVLink2, /dev/ttyACM0)    ┌──────────────────────── Raspberry Pi 5 ──────────────────────────┐
 │   Pixhawk    │ ───────────────────────────────▶ │ mavlink_link.py (thread)                                         │
 │  ArduPilot   │   telemetry: speed, heading,     │   └─▶ VehicleState ──▶ hud.py (draws 30×/s) ──▶ HDMI ──▶ 5" LCD  │
 │ Rover / Boat │   battery, GPS, mode, mission    │                                                                  │
 │              │ ◀─────────────────────────────── │ controller.py ◀── controls.py ◀── hall buttons (GPIO 5/6/13/19)  │
 └──────────────┘   commands: set mode, change     │   └─ throttle.py: RC throttle override, 10×/s                    │
                    speed, RC throttle override    └──────────────────────────────────────────────────────────────────┘
```

- **`mavlink_link.py`** talks to the Pixhawk on a background thread. It asks the Pixhawk to stream the messages we need, downloads the mission, and keeps one `VehicleState` up to date. Commands from the UI are queued and sent from this thread.
- **`hud.py`** reads a snapshot of that state 30 times a second and draws the whole screen.
- **`controls.py`** turns GPIO button edges into `pressed`/`released` events. **`controller.py`** decides what each button does.
- **`throttle.py`** acts as the vehicle's "RC transmitter" for the throttle channel (see §5).
- **`sim.py`** is a fake vehicle with the same interface as the real link. It lets you work on the HUD with no Pixhawk attached.

## 2. Hardware and wiring

| Item | Notes |
|---|---|
| Raspberry Pi 5, Raspberry Pi OS (Debian 13 "trixie"), desktop | Logs in automatically as `dpv` into the labwc (Wayland) desktop |
| 5" 800×480 HDMI LCD | Plug-and-play over HDMI on the Pi 5. **Do not run the vendor's `LCD-show` / `LCD5-show` script.** It was written for older Pis and the legacy display driver. It rewrites `config.txt` and can leave you with a black screen. |
| Pixhawk running ArduPilot Rover (boat frame) | Pixhawk's USB port → any Pi USB port |
| Hall-effect push buttons | On Pi GPIO, one per function |

**Pixhawk → Pi over USB.** The Pixhawk appears as `/dev/ttyACM0`. `config.toml` tries the stable name `/dev/serial/by-id/usb-…-if00` first, so the right port is found even if other USB serial devices are plugged in. To see the board's exact name, run `ls /dev/serial/by-id/` with it connected.

- Power the Pixhawk from its own power module. USB power from the Pi is fine on the bench but not for driving.
- Use a short, good-quality cable, and strain-relieve it: if the cable comes loose, the motor stops (see §5).

**Buttons** (BCM numbers, set in `config.toml`)

| Function | GPIO | Header pin |
|---|---|---|
| MODE | 5 | 29 |
| SPEED | 6 | 31 |
| TRIGGER (dead-man) | 13 | 33 |
| GUIDE | 19 | 35 |
| Ground | – | 30, 34 or 39 |

By default each input uses the Pi's internal pull-up and counts as "pressed" when the sensor pulls it **low**. That suits open-drain hall switches such as the US5881 or A3144. If your sensor drives its output high instead, set `active_high = true`. ⚠️ **A GPIO pin must never see more than 3.3 V.** Power the sensors from 3.3 V, or make sure the output is open-drain.

## 3. Reading the HUD

| Where | What |
|---|---|
| **Top: heading tape** | A compass ribbon. The box shows the current heading. The **bug** on the tape marks the bearing to the selected guidance target. When the target is off the tape, an arrow at the edge points toward it. |
| **Left: speed tape** | Ground speed (knots by default). In AUTO/RTL a magenta bug marks the commanded speed. |
| **Right: battery** | Remaining %, voltage and current. Green above 40 %, amber above 20 %, **red and flashing** at 20 % or less. |
| **Centre: steering cue** | Chevrons point the way to turn, and the number is the error in degrees. 1 chevron means under 20°, 2 means under 60°, 3 means 60° or more. Within 5° it shows **ON COURSE**; within 3 m of the target it shows **AT HOME** / **AT WP n**. |
| **Centre: guidance box** | The selected target (**HOME** in cyan, or **WP n** in magenta) with its bearing, distance, coordinates and ETA. The HOME/WP tabs show which target is selected. |
| Under the heading tape | The latest message from the autopilot (shown for 10 s). Red = error, amber = warning. |
| **Bottom: MODE** | The current mode. A mode that is selected but not yet sent flashes amber (see §4). |
| **Bottom: SPEED SET** | The selected speed level. In AUTO/RTL it also shows the speed this works out to. |
| **Bottom: MOTOR / THR / GPS / LINK** | Armed state, actual throttle output (lit green while the trigger is held), GPS fix and satellite count, and the link to the Pixhawk. |
| **NO LINK** banner | No heartbeat from the Pixhawk for 2 s. Readings are blanked rather than left showing stale values. |

Colours follow cockpit convention: green = normal, cyan = home and selections, magenta = active route, amber = caution, red = warning.

## 4. Buttons

| Button | Press | Notes |
|---|---|---|
| **MODE** | Steps the *pending* mode MANUAL → AUTO → RTL → … | The pending mode flashes. It is only sent to the Pixhawk after **1.5 s with no further presses**, so one accidental bump can't trigger RTL. To get to RTL from MANUAL, press twice and wait. |
| **SPEED** | Steps the level 25 → 50 → 75 → 100 % → 25 … | In MANUAL this is the throttle used while the trigger is held. In AUTO/RTL it is sent straight to the autopilot as level × `WP_SPEED`, and it is re-sent every time AUTO/RTL is entered. |
| **TRIGGER** | Hold to drive, release to stop | Dead-man switch, **MANUAL mode only**. In AUTO/RTL the autopilot controls the throttle. |
| **GUIDE** | Switches between HOME and WP | **HOME** = the launch point (set by the Pixhawk at arming). **WP** = the active waypoint of the mission uploaded from the land station (Mission Planner / QGC). It works in MANUAL too, so the driver can steer to a waypoint by hand. |

For desk testing, the keyboard does the same: **M** = mode, **S** = speed, **Space** = trigger (hold), **G** = guide, **Ctrl+Q** = quit.

## 5. Safety: how the throttle works

The Pixhawk has **no RC receiver**. The Pi stands in for it on the throttle channel only. While the link is up, the Pi sends an `RC_CHANNELS_OVERRIDE` message **10 times a second**:

- in **MANUAL** with the **trigger held**: `neutral + level % × (full − neutral)`. The defaults are 1500 to 2000 µs, so 50 % = 1750.
- **at any other time**: `neutral` (motor stopped).

Every other channel is marked "ignore", so the Pi never touches steering.

If the app crashes, the Pi freezes or the cable comes off, the overrides stop. ArduPilot drops them after `RC_OVERRIDE_TIME` and treats the situation as radio loss, which triggers its throttle failsafe (`FS_ACTION`, set to Hold = stop). For this reason the Pi keeps sending *neutral* rather than going silent when the trigger is released: going silent would look like radio loss and switch the vehicle to Hold every time the driver let go.

> ⚠️ **Do the first powered tests with the propeller removed or the motor unloaded.** Then confirm three things in the ground station: the SERVO3 output follows the trigger and level; releasing the trigger returns it to neutral; and `pkill -f dpv_dash` stops the motor within about half a second. To use the dashboard for display only, set `[throttle] enabled = false`.

## 6. ArduPilot parameters (set from the land station)

| Parameter | Value | Why |
|---|---|---|
| `SERIAL0_PROTOCOL` | 2 | MAVLink2 on USB (this is the default) |
| `SYSID_MYGCS` (called `MAV_GCS_SYSID` on newer firmware) | 255 | ArduPilot only accepts RC overrides from this system ID; the Pi uses 255. If the land station also sends joystick overrides, give one of them a different ID. |
| `RC_OVERRIDE_TIME` | 0.5 | Overrides expire 0.5 s after the last message |
| `FS_THR_ENABLE` | 1 | Enable the radio/throttle failsafe |
| `FS_ACTION` | 2 | Failsafe action = Hold (stop) |
| `RCMAP_THROTTLE` | 3 | Throttle is RC channel 3 (`[throttle] channel`) |
| `WP_SPEED` | your cruise speed | 100 % speed level in AUTO/RTL |

Arming is done from the land station for now. Check that the pre-arm checks pass while the Pi is connected, since its overrides count as the RC input.

## 7. Install and autostart

```bash
cd /home/dpv/dpv_gui
python3 -m venv --system-site-packages .venv     # reuses apt's PyQt5 + gpiozero
.venv/bin/pip install pymavlink

sudo scripts/setup_pi.sh       # disables screen blanking, checks the dialout group
cp deploy/dpv-dash.desktop ~/.config/autostart/
sudo reboot
```

At login, the desktop starts `scripts/run.sh`. That script launches the dashboard full-screen and restarts it 2 s after any crash.

| To… | Run |
|---|---|
| Stop it now | `pkill -f scripts/run.sh; pkill -f dpv_dash` |
| Stop it starting at boot | `touch ~/.config/dpv_dash.disabled` (delete the file to re-enable) |
| Start it again by hand | see §8, step 4 |
| Read the log | `tail -f ~/.local/state/dpv_dash.log` |

## 8. Running it

There are two ways to run the dashboard:

- **Operation mode**: the real thing. It talks to the Pixhawk over USB, and it is what starts automatically at boot.
- **Sim mode** (`--sim`): a fake vehicle that drives around, drains its battery and follows waypoints. Use it to try the HUD and the buttons with no Pixhawk attached. In sim mode **nothing is sent to the Pixhawk**, even if one is plugged in.

All commands below are run from the project folder:

```bash
cd /home/dpv/dpv_gui
```

### Step 1: stop the dashboard that is already running

After boot, the dashboard is already running. Stop it before starting another copy. Otherwise two windows will compete for the screen and for the button GPIO pins.

```bash
pkill -f scripts/run.sh; pkill -f "python -m dpv_dash"
```

### Step 2: start it

From a terminal on the Pi or from an SSH session (either way, it appears on the LCD):

```bash
.venv/bin/python -m dpv_dash --sim        # sim mode
.venv/bin/python -m dpv_dash              # operation mode (Pixhawk on USB)
```

Over SSH the app finds the Pi's desktop session on its own. The desktop must be logged in, which happens automatically at boot.

Useful options (they work in both modes):

| Option | Effect |
|---|---|
| `--windowed` | Normal 800×480 window instead of full screen |
| `--conn URL` | Use a different MAVLink connection, e.g. `--conn /dev/ttyACM1`, or `--conn udpin:0.0.0.0:14550` for ArduPilot SITL on a laptop |
| `-v` | Verbose logging |
| `--test-buttons` | No HUD; prints every hall-button press and release, to check the wiring |

### Step 3: drive it

The hall buttons work in both modes. In sim mode you can also use a keyboard plugged into the Pi; key presses typed over SSH don't reach the window.

| Key | Same as | In sim mode |
|---|---|---|
| **M** | MODE | Wait 1.5 s for the mode to apply. In AUTO the fake vehicle heads for its waypoints; in RTL it heads home. |
| **S** | SPEED | Changes the speed level |
| **Space** (hold) | TRIGGER | In MANUAL, the fake vehicle accelerates while it's held |
| **G** | GUIDE | Switches between HOME and WP |
| **Ctrl+Q** | – | Quit (from SSH: Ctrl+C in the terminal) |

In operation mode, the dashboard shows **NO LINK** until the Pixhawk is plugged in and sending heartbeats. It finds the Pixhawk again automatically if the USB cable is reconnected.

### Step 4: go back to normal

Reboot, or start the autostart launcher again. The launcher always runs operation mode, full screen, and restarts it if it crashes:

```bash
nohup scripts/run.sh >/dev/null 2>&1 &
```

## 9. Development

To take a screenshot of the LCD over SSH: `XDG_RUNTIME_DIR=/run/user/1000 WAYLAND_DISPLAY=wayland-0 grim shot.png`. The log of the autostarted dashboard is at `~/.local/state/dpv_dash.log`.

**`config.toml`**: the Pixhawk device (a path, glob or list), button pins and polarity, speed levels, throttle channel and PWM range, the mode cycle and its commit delay, and the speed units (`kn`, `m/s` or `km/h`).

```
dpv_dash/
  __main__.py      entry point, wires everything together
  state.py         VehicleState (telemetry) + UiState (driver selections) + nav maths
  mavlink_link.py  Pixhawk link thread: streams, mission download, commands
  controller.py    button → action table
  controls.py      GPIO hall buttons (gpiozero)
  throttle.py      dead-man trigger → RC throttle override
  hud.py           all drawing
  sim.py           fake vehicle
scripts/run.sh       autostart launcher with restart loop
scripts/setup_pi.sh  one-time OS setup (screen blanking, dialout group)
deploy/              autostart .desktop entry
```

## 10. Adding a button

1. Wire it and add a pin under `[buttons]` in `config.toml`, then add its name to `BUTTON_NAMES` in `dpv_dash/controls.py`.
2. Write the action as a method in `dpv_dash/controller.py` and register it in `self.actions` (on press) or `self.release_actions` (on release).
3. Optionally, map a key in `KEYMAP` in `dpv_dash/hud.py` for desk testing.
