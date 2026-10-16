# Hardware in the loop (HITL)

The real Pixhawk 6C Mini runs the real firmware: EKF2, the rate and attitude controllers, control allocation, the
failsafes and the uXRCE-DDS link. It flies the same Gazebo F450 and rendered camera that SITL uses. Only the
airframe's sensors and rotors are simulated. The ROS 2 stack runs on the laptop, exactly as in SITL.

## What runs where

A HITL run is three processes on the laptop and a flight controller that outlives all of them:

| Launch / binary | Stands in for | Contents |
| --- | --- | --- |
| `hitl_world.launch.py` | the real world | Gazebo (F450, sensors, camera, `/clock`), the scenario (target, disturbances), `vicon_sim`, and `hitl_bridge`: gz sensors → `HIL_SENSOR` over USB, the FC's `HIL_ACTUATOR_CONTROLS` → gz rotors |
| `MicroXRCEAgent` | the Pi's agent | uXRCE-DDS on the TELEM2 USB-UART: `/fmu/in/*` and `/fmu/out/*` |
| `hitl_flight.launch.py` | the Pi | what `hardware.launch.py` runs on the aircraft: `px4_state_adapter`, `vicon_px4_bridge`, `sim_pilot`, `px4_offboard_bridge`, takeoff gate → estimation → `ibvs_gate` → `pos_ctrl` |

```
hitl_world.launch.py
  Gazebo (F450 + IMU/mag/baro/navsat, camera) ─ hitl_bridge ─ USB ───┐
  scenario (target, disturbances), vicon_sim                         │  Pixhawk, SYS_AUTOSTART 1010
                                                                     │  (SYS_HITL 1)
MicroXRCEAgent ─ USB-UART ─ TELEM2 ──────────────────────────────────┘
        │ /fmu/in, /fmu/out
hitl_flight.launch.py
  px4_state_adapter, vicon_px4_bridge, sim_pilot, px4_offboard_bridge,
  takeoff gate → estimation → ibvs_gate → pos_ctrl
```

`hitl_flight.launch.py` can start the agent itself (`agent:=true`, the default, which `run_hitl.py` uses). Below,
the agent runs in its own terminal (`agent:=false`) so that you can watch the link.

### Constraints

**Two links are needed.**
- USB carries the HIL MAVLink: about 250 `HIL_SENSOR` messages a second in, and `HIL_ACTUATOR_CONTROLS` out.
- uXRCE-DDS cannot use USB. PX4 offers it only on TELEM1/2 and GPS1/2.
- The SiK radio is too slow for DDS. It is still a fine QGC link.

**HITL is not lockstep.** The FC stamps every sensor sample with its own clock on arrival, so Gazebo must run in
real time. `hitl_bridge` withholds `ready` until the real-time factor has held `min_rtf` (0.95) for 5 s. On the dev
laptop, RTF measured 0.999–1.000 both headless and with the GUI.

**PX4 ships HITL only for Gazebo Classic and jMAVSim.** `hitl_bridge` (`quad_gz_sim/scripts/hitl_bridge.py`) is this
project's Harmonic equivalent. Its sensor conversions mirror the fork's `GZBridge.cpp`, so the FC sees what PX4 SITL
sees.

`px4_offboard_bridge` runs with `stamp_clock:=system` here. The nodes run on gz `/clock`, while uXRCE-DDS maps inbound
stamps from the agent host's OS clock onto the FC's, so a sim-time stamp would be meaningless to it.

## Wiring (props OFF)

- **USB:** Pixhawk to the laptop. It enumerates as `/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00`.
- **TELEM2:** the three jumpers (TX, RX, GND) that go to the Pi's UART in flight. In HITL they go to a 3.3 V USB-UART
  adapter on the laptop: `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0` (`/dev/ttyUSB0`).
  - Cross TX and RX.
  - Leave the adapter's 5 V unconnected.
  - With no RTS/CTS, flow control stays off (`UXRCE_DDS_FLCTRL 0`, the default).
- **RC receiver:** stays on the S.BUS RC port. It is only used with `pilot:=rc`.
- **SiK:** stays on TELEM1 for QGC. This is optional, because `hitl_bridge` also forwards QGC over UDP 14550.
- **Power:** USB alone powers the FC. In HITL nothing drives the real outputs (`pwm_out` and `dshot` are not
  started), but the props stay off anyway.

Your user must be in `dialout`; it already is.

## One-off setup

### The workspace and the agent

```sh
cd ~/Robotics/fxteso_ibvs
git submodule update --init --recursive

# MicroXRCEAgent, pinned at v3.0.1; the superbuild fetches its own Fast-DDS (a few minutes)
cmake -S external/Micro-XRCE-DDS-Agent -B external/Micro-XRCE-DDS-Agent/build
cmake --build external/Micro-XRCE-DDS-Agent/build -j$(nproc)

colcon build --symlink-install --allow-overriding cv_bridge image_geometry
pip install pymavlink          # hitl_bridge; the gz Python bindings come with Harmonic
```

The launch files find the agent at `external/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent`. To use another binary, pass
`xrce_agent:=<path>`.

### Firmware and airframe

1. **Back up the parameters.** Changing the airframe resets everything except `CAL_*`, including the RC calibration
   and the `RC_MAP_*` switches. A backup of the v1.17.0 parameters, taken before the first flash, is at
   `~/px4_backup/fmu-v6c_v1.17.0_2026-10-05.params`.
2. **Build and flash the fork.** Its `msg/` definitions are byte-identical to `src/px4_msgs`, so the message versions
   match. Close anything holding the USB port first (QGC, `hitl_bridge`).
   ```sh
   make -C external/PX4-Autopilot px4_fmu-v6c_default upload
   ```
   With QGC you can instead use *Firmware → Advanced → Custom firmware file* and pick
   `external/PX4-Autopilot/build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4`.
3. **Select the airframe** "HIL F450 (FxTESO IBVS)", id **1010**: in QGC, *Airframe → Simulation*, or in the
   MAVLink shell:
   ```
   param set SYS_AUTOSTART 1010
   param set SYS_AUTOCONFIG 1
   reboot
   ```
   The airframe (`ROMFS/px4fmu_common/init.d/airframes/1010_f450_quad_x.hil`) mirrors SITL's `22100_gz_F450_px4`:
   - `SYS_HITL 1`;
   - the F450 rotor geometry, and `HIL_ACT_FUNC1-4` in SITL's motor order;
   - the F450 rate gains;
   - zero declination;
   - `UXRCE_DDS_CFG 102` (TELEM2) at 921600;
   - `SDLOG_MODE 1`.

Everything that depends on launch arguments (`MPC_THR_HOVER`, `MIS_TAKEOFF_ALT`, the EKF2 vision set,
`COM_RC_IN_MODE`, …) is pushed by `hitl_bridge` at start-up. It reads every value back and reboots the FC once if any
of them changed. The set comes from `px4_params()` in `quad_px4/launch/px4_stack.py`, the same resolver that feeds
SITL's `PX4_PARAM_*`.

## Running it, terminal by terminal

Before starting:
- Props off.
- QGC must not hold the USB port. `hitl_bridge` needs it, and QGC reaches the FC through `hitl_bridge` on UDP 14550.
- Nothing from an earlier run should still be up: `pgrep -af "gz sim|MicroXRCEAgent|ros2 launch"` should print
  nothing.

**In every terminal**, first:
```sh
cd ~/Robotics/fxteso_ibvs
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

**Pick the profile once.** Set `ARGS` in T1 and T3 to the same bash array; [Profiles](#profiles-flown-on-hitl) lists
the tested ones. It is an array, with `wind_velocity` single-quoted, so that bash never globs `[x,y,z]`. Both launches derive
the PX4 parameter set on their own, and `hitl_wait` refuses to start if they disagree, naming the parameter.
```sh
ARGS=()                                              # the default: hover, calm air
```

### T1 — the world

```sh
ros2 launch quad_px4 hitl_world.launch.py "${ARGS[@]}"
```
Wait for these, in order:
1. `SYS_AUTOSTART` 1010 and `SYS_HITL` 1 are confirmed.
2. The parameters are pushed. If any changed, the FC reboots once.
3. **"FC parameters verified"**.
4. **"HITL ready (RTF 1.00)"**: `HIL_ACTUATOR_CONTROLS` are flowing and RTF has held for 5 s.

`ros2 topic echo /hitl_bridge/status` shows `"ready": true`. QGC is optional; open it now and it auto-connects on
UDP 14550.

### T2 — the agent on TELEM2

```sh
external/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent serial \
  --dev /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0 -b 921600
```
You should see `session established`, then a burst of `create_topic` / `create_datawriter` lines.

### T4 — check the DDS link

```sh
ros2 topic hz /fmu/out/vehicle_odometry               # ~100 Hz
ros2 topic echo --once /fmu/out/vehicle_status_v4
```

### T3 — the flight stack

The stack arms on its own once `hitl_wait` passes.
```sh
ros2 launch quad_px4 hitl_flight.launch.py agent:=false rosbag:=true "${ARGS[@]}"
```
1. `hitl_wait` exits 0.
2. `sim_pilot` arms and climbs to about 1.5 m.
3. `px4_takeoff_gate` passes and the estimators start; `ibvs_gate` holds a marker lock.
4. `pos_ctrl` starts. That is the handover: `px4_offboard_bridge` logs `Streaming -> Offboard` at zD 1.2, and the
   bag starts recording.
5. A moving target is released once the loop has held it centred. Any wind starts after the handover.

`controllers:=false` stops after step 3. The aircraft just loiters, which is a good first dry run.

While it flies, from T4:
```sh
ros2 topic echo /vicon_px4_bridge/aiding_active                               # true
ros2 topic echo --once /fmu/out/estimator_status_flags | grep -E 'cs_ev_(pos|yaw)'
ros2 topic echo /fmu/out/vehicle_status_v4 --field nav_state                  # 14 = OFFBOARD
```

### Ending a run

The FC outlives every launch, so a run can't end the way SITL does, by killing PX4. Land it first. The campaigns flew
200 s after the handover.

1. **Stop `px4_offboard_bridge` alone,** from T4. Its OFFBOARD recovery would otherwise fight the LAND.
   ```sh
   pgrep -af lib/quad_px4/px4_offboard_bridge          # confirm it is the one
   pkill -TERM -f lib/quad_px4/px4_offboard_bridge
   ```
   PX4 drops to Altitude (`COM_OBL_RC_ACT 1`).
2. **Land it with T2 still up.** Run
   `python3 ~/Robotics/fxteso_ibvs_experiments/experiments/hitl_land.py`, which sends `NAV_LAND` over DDS and waits
   for the auto-disarm, or press QGC's **Land**.
3. Once it is disarmed, press Ctrl-C in T3, then T2, then T1.

After disarm, `vehicle_status.nav_state` reads 14 again: that is the user intention, not OFFBOARD.

The bag is in `bags/hitl_<stamp>/`. The FC's ulog is on its SD card: QGC *Analyze → Log Download*, or
`hitl_ulog.py --latest 1 --out bags/hitl_ulogs` in the experiment repo, with T1 stopped so that the USB port is free.

For campaigns, the experiment repo's `run_hitl.py` takes a `run_matrix.py` spec and does all of this per run. It
reboots the FC first, flies, lands, and pulls the run's ulog.

### If something fails

| Symptom | Cause | Fix |
| --- | --- | --- |
| T3's nodes sit at sim time 0, and the gates time out | `/clock` never reached them; usually a wedged ros2 daemon | `ros2 daemon stop`, then restart T3. Check with `ros2 topic echo --once /clock` |
| `hitl_wait` refuses, naming a parameter | T1 and T3 were given different arguments | Use the same `ARGS` in both |
| T1 never reports ready | RTF below `min_rtf` | `headless:=true` in T1 only. It is not a PX4 parameter, so it may differ between terminals |
| No `session established` in T2 | TELEM2 wiring or port | TX/RX crossed? `UXRCE_DDS_CFG 102` on the FC? Right `--dev`? |
| `hitl_flight` fails with `/dev/ttyUSB0 not found` | `agent:=false` was forgotten and the adapter is unplugged | Pass `agent:=false`, or plug the adapter in |

## Profiles flown on HITL

These are the target and disturbance profiles the real FC has flown. All use the default base: zD 1.2 after a 1.5 m
takeoff, `venue:=vicon aiding_policy:=always velocity_source:=off`, the 1.30 kg 3S plant, `module3wide_2304`, and the
hybrid detector.

The two image axes differ:
- the narrow axis is world +x, with a 0.62 m budget;
- the wide axis is +y, with a 1.31 m budget. A moving target travels along it (`target_heading` 90).

"Held" means the loop kept the target to the end of the window without losing authority. Every arm below was flown
the same way in SITL, with matching results.

| Profile | `ARGS=(…)` | Held on HITL |
| --- | --- | --- |
| Hover, calm air | `()` | 1/1, 234 s (2026-10-09) |
| Line, 0.7 m/s | `(target_profile:=line target_speed:=0.7)` | 3/3 (E102) |
| Thesis staircase | `(target_profile:=thesis)` | 3/3 (E102) |
| Line + wind 2.68 m/s | `(target_profile:=line target_speed:=0.7 disturbance:=wind wind_scale:=3.0)` | 3/3 (E102) |
| Thesis + wind 2.68 m/s | `(target_profile:=thesis disturbance:=wind wind_scale:=3.0)` | 9/10 (E102 + E103) |
| Hover, 3.0 m/s wind step on the narrow axis | `(disturbance:=wind 'wind_velocity:=[3.0,0.0,0.0]')` | 3/3 (E103) |
| Line, 3.0 m/s wind step on the narrow axis | `(target_profile:=line target_speed:=0.7 disturbance:=wind 'wind_velocity:=[3.0,0.0,0.0]')` | 3/3 (E103) |
| Hover, 5 m/s narrow wind ramped over 60 s | `(disturbance:=wind 'wind_velocity:=[5.0,0.0,0.0]' wind_ramp:=60)` | 3/3 (E104c) |
| Line, 5 m/s narrow wind ramped over 60 s | `(target_profile:=line target_speed:=0.7 disturbance:=wind 'wind_velocity:=[5.0,0.0,0.0]' wind_ramp:=60)` | 3/3 (E104c) |

Notes:
- `wind_scale:=3.0` scales the default `wind_velocity` `[0.8, 0.4, 0.0]`, giving 2.68 m/s, mostly along the narrow
  axis.
- 3.5 m/s ramped over 60 s also held, 3/3 for hover and for line.
- A step switches the full wind on 8 s after the controllers start. `wind_ramp:=T` instead ramps the mean wind up over
  `T` seconds.
- The wind limit depends on how fast the force rises, not on its level, so always quote the ramp with the speed.

**Known to fail on HITL** (matching SITL):
- a narrow-axis step at 3.5 m/s (0/6) or 4.0 m/s (0/6);
- 7 or 10 m/s over a 60 s ramp (0/6 each);
- 10 m/s over 240 s (0/3 hover, 0/3 line).

**Not flown on HITL yet:**
- `disturbance:=step|gust|table52|csv`;
- wind along the wide axis;
- the 480 s ramp, which held 10 m/s in SITL;
- `pilot:=rc`.

These are only SITL results, or none at all; see [RUNNING_SIM.md](RUNNING_SIM.md#disturbance-profiles).

## Argument reference

| Argument | Default | Notes |
| --- | --- | --- |
| `pilot` | `sim` | `sim`: `sim_pilot` flies the takeoff over MAVLink sticks (`COM_RC_IN_MODE 1`). `rc`: your transmitter flies it and the handover needs consent (`COM_RC_IN_MODE 0`, `MAN_OVERRIDE_SPD 1.0`). Untested. |
| `consent_rc_aux` | `0` | `pilot:=rc`: RC AUX channel for consent (needs `RC_MAP_AUXn`). 0 means the `~/handover` service. |
| `fc_device` | by-id path | `hitl_world`. `none` runs it without an FC, to check RTF only. |
| `min_rtf` | `0.95` | `hitl_world`. |
| `gcs_udp_port` | `14550` | `hitl_world`. QGC auto-connects to the forwarded link. 0 disables it. |
| `agent` | `true` | `hitl_flight`. `false`: start no agent, because you run `MicroXRCEAgent` yourself (T2). |
| `agent_device`, `agent_baud` | `/dev/ttyUSB0`, `921600` | `hitl_flight`, `agent:=true` only. The TELEM2 USB-UART. |
| `xrce_agent` | the submodule's build | The `MicroXRCEAgent` binary. |

Every `sitl.launch.py` argument also applies, with the same defaults ([RUNNING.md](RUNNING.md#launch-arguments-reference)).
Disturbances act on the gz body (BodyWrench) and need nothing HITL-specific. `vicon_sim` publishes from gz truth in
`hitl_world`, and `vicon_px4_bridge` sends it to EKF2 over the real link from `hitl_flight`.

## Bring-up checks, for a new FC or new firmware

1. **No FC.** `ros2 launch quad_px4 hitl_world.launch.py fc_device:=none`. Read the RTF from `/hitl_bridge/status` or
   `gz topic -e -t /stats`.
2. **Firmware.** The parameter readback shows `SYS_HITL 1`, `UXRCE_DDS_CFG 102` and the F450 `CA_ROTOR*`.
3. **HIL link.** T1 only:
   - the bridge logs "FC parameters verified";
   - QGC on UDP shows EKF2 aligned and no preflight failures;
   - the attitude follows the gz model.
4. **DDS link.** T2, then T3 with `controllers:=false`:
   - `vehicle_odometry` arrives at about 100 Hz;
   - `cs_ev_pos` and `cs_ev_yaw` are set;
   - `aiding_active` is true.
5. **Takeoff and loiter** with `sim_pilot`.
6. **Full loop.** Hover at zD 1.2 with `rosbag:=true`, compared with SITL on station-keeping, authority gaps and
   OFFBOARD ownership.

## Logging

At boot the logger fixes its topic list and skips optional topics that don't exist yet. EKF2's `estimator_*` topics
only appear once HIL sensors flow. An FC booted with no `hitl_bridge` attached therefore logs no estimator topics at
all, and even when attached the default profile logs them at 2 Hz. `/fs/microsd/etc/logging/logger_topics.txt` on the
SD card subscribes them unconditionally at 100 Hz. Write it with `hitl_ulog.py --shell`, and remove it with
`rm /fs/microsd/etc/logging/logger_topics.txt`.

HITL also pushes SITL's `IMU_INTEG_RATE 250` and `SDLOG_PROFILE 131` (the FC defaults are 200 and 1).

## Leaving HITL

`SYS_HITL` lives in the FC's saved parameters. Before any real flight:
1. Select the flight airframe (`SYS_AUTOSTART 4001` was the previous one, or the F450 frame you fly) with
   `SYS_AUTOCONFIG 1`, then reboot.
2. Load `src/quad_px4/params/hardware.params`.
3. Restore the RC calibration and switches from the backup, or redo them in QGC.
4. Check `SYS_HITL 0`, real sensors present, and calibrations valid.
5. Optionally remove `/fs/microsd/etc/logging/logger_topics.txt`. It only adds logging, so it is harmless in flight,
   but the logs are larger.
