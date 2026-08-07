# fxteso_ibvs

Image-based visual servoing of a quadrotor with a **fixed-time extended state observer**
(FxTESO) and an **adaptive-gain sliding-mode controller**. ROS 2 Jazzy / Gazebo Harmonic port
of the ROS 1 `fxteso_asgibvs` package.

**The control law, the observer and the plant model are unchanged from the ROS 1 original.**
Only the simulation and middleware layers were rewritten. Gains, loop rates and topic names are
identical, so results diff cleanly against the thesis.

```
ros2 launch fxteso_ibvs sim.launch.py [headless:=true] [rosbag:=true] [foxglove:=true]
                                      [disturbance:=none|step|gust|wind|csv]
                                      [disturbance_seed:=N]
```

---

## 1. What Gazebo is, and what it is not

**Gazebo is a camera renderer. It is not the physics engine, and it does not simulate the
quadrotor.** This is the single most surprising fact about this package, and almost every
confusion about it starts here.

- `worlds/ibvs.sdf` sets `<gravity>0 0 0</gravity>` and `<physics type="ignored">`.
- `gz_pose_broadcaster` overwrites the pose of **both** the quad and the target every 10 ms
  from ROS-side state, via the gz-transport service `/world/ibvs/set_pose_vector`. Nothing in
  Gazebo integrates anything.
- The **only** signal that flows back from Gazebo into the control loop is the rendered camera
  image, `/quad/camera/image_raw`.

Consequences worth internalising:

- Adding a Gazebo wind plugin, a lift/drag plugin, or rotor thrust plugins would do nothing.
  Disturbances must be injected on the ROS side (§4).
- The F450 model's rotors, inertias and collision geometry are decorative. Only the camera
  sensor (`models/F450/model.sdf:212`) matters.
- The physics system plugin is still loaded, because the pose-update path runs through it —
  `type="ignored"` alone makes model teleports never reach the render scene.

`config/bridge.yaml` is one-way (`GZ_TO_ROS`) and carries exactly three things: the image, its
`camera_info`, and `/clock`.

## 2. The plant

`src/uav_dynamics.cpp` is the simulated aircraft: a 6-DOF rigid body integrated with **forward
Euler at 100 Hz**.

| | |
|---|---|
| mass | 2 kg |
| inertia | `diag(0.0411, 0.0478, 0.0599)` |
| initial position | `(-9.9, -10.1, -4)`, 0.14 m lateral of the target, 4 m above |
| inputs | `/quad_thrust` (scalar, N) and `/quad_torques` (body torque, N·m) |
| disturbance | `/disturbances`, applied as `-R(η)ᵀ·d` in the body force |

There is **no aerodynamic drag, no rotor dynamics, no motor time constant and no mixer** —
thrust and body torque are commanded directly. Ground contact is a hard clamp at `z > 0`. This
is a control-design plant, not a high-fidelity airframe model; do not read absolute flight
performance off it.

## 3. Node graph

```
  target_position ──> tgt_position / tgt_yaw / tgt_velocity / tgt_acceleration
        │
  uav_dynamics ────> quad_position / quad_attitude / quad_velocity / quad_velocity_BF
        │                                    │
        │            gz_pose_broadcaster ────┴──> Gazebo (renderer only)
        │                                              │
        │                       [ros_gz_bridge] <── /quad/camera/image_raw
        │                                              │
        v                                        image_features
  td_linear / td_attitude / td_attitude_desired        │  ImFeat_vector
        │                                              │
        └──────────> fixed_eso ──> pos_ctrl ──> att_ctrl ──> quad_torques / quad_thrust
                                       │                          │
                                       └── ImFeat_estimates_fxt, ibvs_dist
```

| node | rate | start-up hold | role |
|---|---|---|---|
| `uav_dynamics` | 100 Hz | 5.0 s | the plant |
| `disturbances` | 100 Hz | 8.0 s | disturbance force (§4) |
| `target_position` | 100 Hz | 0.05 + 2.3 + 2.0 s | moving target trajectory |
| `image_features` | 50 Hz | 1.7 s | ArUco → feature vector |
| `td_linear` | 100 Hz | 5.0 s | tracking differentiator, position/velocity |
| `td_attitude` | 100 Hz | 4.5 s | tracking differentiator, attitude |
| `td_attitude_desired` | 50 Hz | 4.5 s | tracking differentiator, desired attitude |
| `fixed_eso` | 50 Hz | 4.7 s | **FxTESO** |
| `pos_ctrl` | 50 Hz | 1.0 s | adaptive-gain SMC, IBVS outer loop |
| `att_ctrl` | 100 Hz | 2.0 s | attitude inner loop |
| `tf_broadcaster` | 50 Hz | — | TF / paths / markers for Foxglove |

### The observer is in the loop

`fixed_eso` is not instrumentation. `pos_ctrl` takes its **entire error signal** from the
observer:

- `error = imgFeat_est - imgFeat_des` — `aibvs_pos_ctrl.cpp:410`
- `error_dot = imgFeat_dot_est` — `aibvs_pos_ctrl.cpp:411`
- `ibvs_dist` is fed forward in all four channels — `aibvs_pos_ctrl.cpp:429-436`

The raw `/ImFeat_vector` reaches `pos_ctrl` only inside the block marked
`FOR COMPARISON ONLY` (`:389-407`), which computes `/real_err_dot` for plotting and feeds
nothing back. Likewise `pos_ctrl` and `att_ctrl` consume the **tracking-differentiator
estimates** (`attitude_estimates`, `lin_vel_BF_estimates`), not the plant's own outputs.

### Image features

`image_features.cpp` detects four `DICT_7X7_50` ArUco markers at 50 Hz and forms the feature
vector `(qx, qy, qz, qψ)`. When it sees anything other than exactly four markers it publishes
the sentinel **`(0, 0, 1, 0)`** and prints `I see N arucos only`. That sentinel is a
structurally valid feature vector, which is why start-up needs a gate (§6).

The camera is 820×616 at 50 Hz — twice the ROS 1 resolution, needed to decode the markers from
the 4 m starting altitude; `pixel_size` in `image_features.cpp` is halved to match.

## 4. Disturbances

`/disturbances` is an **inertial-frame force in newtons**. `uav_dynamics.cpp:171` applies

```
force_BF = R(η)ᵀ (m g e₃) − thrust·e₃ − R(η)ᵀ d
```

so `d` is neither a wind velocity nor an acceleration. `src/disturbances.cpp` generates it.

**Mind the sign.** Because of that minus, and because the frame is NED (`e₃` points down,
`z = -4` is 4 m up), a **positive** value on `/disturbances` accelerates the quad in the
**negative** direction of that axis. `step` and `gust` are symmetric about zero so it does not
show up there, but it is why the `wind` generator publishes `-F_drag`: publishing `+F_drag`
inverts the `-v_quad` term of the drag law into *negative damping* — a force accelerating the
quad along its own velocity — and the plant diverges for any wind speed at all.

Magnitudes live in **`config/disturbances.yaml`**, loaded by the launch file, so they can be
tuned without touching C++. `profile` and `seed` are deliberately not in that file: they come
from the `disturbance:=` and `disturbance_seed:=` launch arguments, so each parameter has one
source. (Setting them in the YAML silently wins over the command line.)

> **History.** This node used to read its profile from a CSV whose default path pointed at the
> original author's machine (`/home/armando/...`). That file is not in this tree, the open
> failed silently, the `getline` loop ran zero times, and the node published `0 N` for the whole
> run. Every simulation before this change was **undisturbed**, and every "truth vs FxTESO
> estimate" plot showed a flat-zero truth trace that looked like a real measurement. A missing
> profile is now fatal.

### Profiles

`profile` is a **comma-separated set**, not an enum — the generators compose and their outputs
are summed. `mean` is added whenever anything is active.

| profile | parameters | notes |
|---|---|---|
| `none` | — | publishes 0 N. The undisturbed baseline. |
| `step` | `step_amplitude` (N, default `[0.3,0.3,0.3]`), `step_start`, `step_end` (s) | Rectangular pulse. Default window `[22, 25] s` reproduces the thesis' step test. |
| `gust` | `gust_sigma` (N, default `[0.4,0.4,0.2]`), `gust_tau` (s, default 1.5), `seed` | Per-axis Ornstein–Uhlenbeck: stationary, zero-mean, band-limited. `d ← d − (d/τ)Δt + σ√(2Δt/τ)·N(0,1)`. |
| `wind` | `wind_velocity` (m/s), `wind_turbulence` (m/s), `wind_turbulence_tau` (s), `drag_cd_a` (m², default 0.15) | `F = ½ρ·CdA·\|v_rel\|·v_rel` with `v_rel = (v_wind + turbulence) − v_quad`, from `/quad_velocity`. `wind_turbulence` is an OU process **on the wind velocity**, so a gust changes the airspeed and the force follows quadratically. Set it to `[0,0,0]` for a steady wind. |
| `csv` | `csv` (path) | Replays one `x,y,z` row per 10 ms step, header row skipped. Aborts if the file cannot be opened. |

Plus `mean` (N, default `[0,0,0]`) — a constant bias added to all of the above.

The resolved configuration is logged at INFO on start-up, so every run and every bag is
self-describing.

`t = 0` is the **end of the 8 s start-up hold**, not launch. The hold exists because
`uav_dynamics` sits on its initial condition for 5 s and `ibvs_gate` only then releases the
controllers: the plant has to be flying before it is pushed. This matches the timebase the old
hard-coded pulse counted from.

### Which profile for which question

- **Validating the observer** → `step` or `gust`. Their truth signal is independent of the
  state, so `/disturbances` vs `/scaled_ibvs_dist` is a clean comparison.
- **"What does wind do to it"** → `wind`. Note this one is state-dependent (it is a drag law),
  so it also adds velocity damping to the plant, and the truth signal is not independent.
- **Reproducibility** → `gust` and `step` are deterministic for a fixed `seed`; two runs with the
  same seed produce a bit-identical `/disturbances` trace.

### How hard you can push — the field of view is the limit

Measured on 75 s headless runs with the target on its default trajectory:

| disturbance | visual lock | outcome |
|---|---|---|
| none | held | servoing error ~1e-3 |
| `step`, 0.3 N for 3 s | held | error ~0.12, bounded, recovers |
| `gust`, σ = 0.10 N | held | full run |
| `gust`, σ = 0.20 N | **lost** | diverges to NaN |
| `wind`, \|v\| = 1.12 m/s | held | full run, error ~0.10 |
| `wind`, \|v\| = 1.68 m/s | **lost** | diverges to NaN |

A **sustained** disturbance above roughly 0.1–0.2 N displaces the quad far enough that the ArUco
markers leave the 820×616 frame. `image_features` then emits the no-lock sentinel, the
controller servos on it, and the plant diverges within seconds (§8.5). A 0.3 N *step* is fine —
duration, not peak, decides. The log symptom is a flood of
`I see N arucos only. I need 4 to work properly`.

**This ceiling belongs to the vision setup, not to the FxTESO or the SMC.** Raising it means
widening the field of view, raising `z_des`, or adding a loss-of-lock hold — not retuning the
controller. The shipped defaults sit inside the safe band.

### Reading the estimate

`fixed_eso` publishes the raw IBVS-space disturbance on `/ibvs_dist` and a force-scaled version
on `/scaled_ibvs_dist` (`fixed_eso.cpp:287`), directly comparable with `/disturbances`. Plot the
two together in Foxglove — that is the primary observer-validation figure, and it has only been
meaningful since the CSV fix above.

Sampled late in a `wind` run, with the target yawing (so `Ryaw(ψ)` is not identity):

```
/disturbances      x = -0.390   y = -0.146   z = -0.014    (truth)
/scaled_ibvs_dist  x = -0.365   y = -0.127   z = +0.013    (FxTESO estimate)
```

The estimate tracks the truth to within a few percent on the loaded axes, which also confirms
that the `Ryaw(ψ)ᵀ` convention and the sign at `fixed_eso.cpp:287` are **correct as written** —
no transpose or sign change is needed. (One sample, one yaw angle: treat it as strong evidence,
not a proof across the whole trajectory.)

## 5. Models

Both models are visual only — no collision geometry anywhere in the world, and the inertias are
never integrated (§2 owns the plant).

**F450** (`models/F450/model.sdf`), converted from `f450_urdf/F450_assembly.urdf`:

- **Frames.** `gz_pose_broadcaster` applies a π roll to the model — that is what points the
  camera down — so the airframe is built inside the frame `body`, which is the model frame
  rolled by π. Everything in the file is therefore normal FLU: +x forward, +y left, +z up.
  Do not move the roll into the SDF, and do not add a second one.
- The camera keeps the placeholder model's exact optical pose, so `image_features`' calibration
  is untouched. 820×616 at 50 Hz; `pixel_size` in `image_features.cpp` is halved to match.
- **Meshes** are in mm (`scale 0.001`). The four arms/motors/propellers are exact z-rotations of
  each other, so one STL of each is shipped and copies are placed at yaw `k·90°`.
- The CAD exported one propeller part for all four positions, which would push air the wrong way
  at half the rotors. `propeller_cw.stl` is that mesh mirrored about its hub plane.
- **Rotor spin** is cosmetic — there is no aerodynamics, so nothing would turn the joints.
  `JointController` writes joint velocity straight into physics, which keeps working while the
  broadcaster teleports the model. Layout is ArduPilot/PX4 QuadX (front-right and rear-left CCW);
  `gz_pose_broadcaster` repeats these signs, so **change one and you must change the other**.
  Speeds are ~10× slower than a real 880 KV motor because true rotor speed just aliases at
  50 Hz.

**aruco_target** (`models/aruco_target/model.sdf`): a 0.90 × 0.75 m plate carrying four
`DICT_7X7_50` markers, IDs 4, 6, 8, 10. The model name must stay `aruco_Target` —
`gz_pose_broadcaster` addresses the entity by that exact string, and `image_features.cpp` keys
its point assignment off the marker IDs baked into `Target2.dae`. That file references its four
JPEGs by bare filename, so they must sit beside it in `meshes/`.

## 6. Start-up sequencing

Everything is paced on **simulation time**, not wall clock.

`include/fxteso_ibvs/sim_rate.hpp` provides `fxteso::SimRate`, a drop-in replacement for
`rclcpp::Rate` that ticks on the node's clock (`/clock`, bridged from Gazebo). It spins rather
than blocking: these nodes are single-threaded, so `Clock::sleep_until` would stop `/clock`
being serviced and sim time would never advance. **`SimRate` owns the executor — a node using it
must not also call `rclcpp::spin_some`.** Because every node is paced this way, lowering
`real_time_factor` in `worlds/ibvs.sdf` slows the whole stack together and preserves the control
loop's relative timing. That is the correct way to slow the simulation down; lowering the
camera's `update_rate` is not.

`src/ibvs_gate.cpp` holds `pos_ctrl` and `att_ctrl` back until closed-loop servoing is actually
possible: quad placed, target placed, and `/ImFeat_vector` non-sentinel for
`required_lock_frames` (default 10) consecutive frames. It then exits 0, and the launch file's
`OnProcessExit` handler starts the controllers and the rosbag recorder. On timeout (default
120 s, deliberately **wall** clock, so it still fires when Gazebo never starts and `/clock` never
advances) it exits non-zero and the controllers are never started.

This replaced a fixed 7 s `TimerAction`. That delay was wall clock while every node staggers
itself in sim time, and Gazebo spends seconds loading the world before it steps at all — so 7 s
of wall clock was only ~3.6 s of sim time, ahead of the plant's 5 s hold and ahead of the first
marker lock. The controllers would then servo on the no-lock sentinel. How far the clocks
diverged depended on world load time, which is why it only bit sometimes.

## 7. Recording and visualisation

`rosbag:=true` records the topic list in `sim.launch.py:BAG_TOPICS` to `bags/ibvs_<timestamp>`
in **mcap** format (not rosbag2's default sqlite3 — Foxglove Studio cannot open `.db3`). Camera
images are deliberately excluded: 820×616 at 50 Hz is ~75 MB/s and would dwarf every signal of
interest. The recorder starts with the controllers, so the bag has no dead air at the front.

`foxglove:=true` starts `foxglove_bridge`; the layout is in `foxglove/fxteso_ibvs.json` and
`foxglove/check_layout.py` validates that every panel's topics exist.

## 8. Known limitations

These are real and currently unaddressed. They are recorded rather than fixed because the
control and observer algorithms are the subject of study and are deliberately left as-is.

1. **`f(x)` is zero in the observer.** `fixed_eso.cpp:231, 245, 258, 271` hard-code the known
   dynamics to 0, with the correct feature/yaw coupling terms sitting commented beside them
   (`q̈x = ψ̈·qy + ψ̇·q̇y − v̇x/z` and its `y` counterpart). The cross-coupling is therefore
   absorbed into the estimated disturbance rather than modelled, so `/ibvs_dist` during a yawing
   segment is *coupling + disturbance*, not disturbance alone. `pos_ctrl` mirrors this: it
   feed-forwards the `ψ̇·q̇` half (`:432, :434`) while the `ψ̈·q` half is commented at `:439`.
   The two files are consistent with each other as they stand — **if one is ever changed the
   other must change with it**, or the coupling is double-counted.
2. **`z_des` is a hard-coded 2.5** in both `fixed_eso.cpp:56` and `aibvs_pos_ctrl.cpp:90`, while
   the quad starts at 4 m depth. The observer's input matrix `g(x)u = −(1/z)·u` and the
   `/scaled_ibvs_dist` conversion both use nominal, not measured, depth.
3. ~~`/scaled_ibvs_dist`'s frame is unverified.~~ **Resolved.** Checked against a real `wind`
   disturbance during the yawing segment: `/scaled_ibvs_dist` matches `/disturbances` to a few
   percent, so the `Ryaw(ψ)ᵀ` and the sign at `fixed_eso.cpp:287` are correct. Left here
   because it was unfalsifiable for as long as the disturbance was identically zero — which is
   the real lesson.
4. **Integration steps are duplicated literals.** `fixed_eso.cpp` uses `step = 0.02` (50 Hz),
   `uav_dynamics.cpp` and `disturbances.cpp` use `0.01` (100 Hz). Consistent with their loop
   rates today, but nothing enforces that — change a node's `SimRate` and its integrator
   silently goes wrong.
5. **No mid-run loss-of-lock guard — and it is the binding constraint on this simulation.**
   `ibvs_gate` protects start-up against the `(0,0,1,0)` sentinel, but if the markers leave the
   frame during a run the controllers servo on it and the plant diverges to NaN within seconds.
   This, not controller stability, is what caps the disturbance magnitude at ~0.1–0.2 N
   sustained (§4). The cheapest fix would be for `pos_ctrl` to hold its last good feature
   vector, or freeze its output, while `/ImFeat_vector` reads as the sentinel — but that is a
   change to the control path, so it is recorded rather than made.
6. **Forward Euler throughout.** Plant, observer, differentiators and controller all use
   explicit Euler at their own rates. Fine at these step sizes, but it bounds how far the gains
   can be pushed before the discretisation, rather than the theory, is what fails.

## 9. Operational notes

- `sim.launch.py` reaps `gz sim` processes orphaned by a previous run before starting (they hold
  a GPU context). It requires all three of `gz sim` in the cmdline, `fxteso_ibvs` in it, and
  `ppid == 1`, so a live simulation is never touched.
- The EGL vendor environment variables at the top of the launch file force the NVIDIA ICD;
  without them gz-sensors picks Mesa and falls back to software rendering (llvmpipe) even with
  the GPU present.
- `<gui>` is declared in full in `worlds/ibvs.sdf`, which makes gz-sim ignore `gui.config`
  entirely — every plugin the window needs must be listed there.
- System OpenCV on Ubuntu Noble is 4.6.0, which still ships the pre-4.7 `cv::aruco` API, so
  `image_features.cpp` compiles unchanged from the ROS 1 original.
