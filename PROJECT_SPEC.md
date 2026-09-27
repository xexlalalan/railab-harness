# Bunny Harness Project Specification

This document is the specification and configuration guide for the **Bunny Harness** — the workcell that integrates a robotic arm (currently an **Agilex PIPER**), one or more **Weighing Super Bunny** powder-dispensing instruments, and the accessories around them. It follows the same conventions as the instrument's own spec (`~/bunny-dev/PROJECT_SPEC.md`, the `bunny` repository), which stays authoritative for everything *inside* the instrument.

Created 2026-09-15. Status: **initial draft** — most sections are PROPOSED or TBD and exist to be argued with; the "Bring-up Log" and "Development Environment" sections record verified facts.

> **Naming**: **Bunny Harness** (this project; to be renamed **railab-harness** once it is a general platform — decided 2026-09-15) is the *system*: arm + instrument(s) + accessories + the software that orchestrates them. **Weighing Super Bunny** is the *instrument* (repo `bunny`, the 1→N dispenser). **PIPER** is the *arm* (Agilex, 6-DOF). The Bunny spec calls this project "the workcell repo" / "system-level integration repo" — same thing.

## Prompt Guidelines

**CRITICAL: This section must be read and extracted at the beginning of every development session.**

### Core Development Directive

Before any modification to the code, a mandatory clarification process must be followed:

1. **Extract and Read**: Always extract and read this Prompt Guidelines section first when starting any development work
2. **Clarification Before Implementation**: Engage in comprehensive back-and-forth discussion to resolve ALL ambiguities before writing any code
3. **Ask Questions**: Actively inquire about unclear requirements, missing information or context, implementation approaches that could vary, edge cases not explicitly covered, and integration with existing functionality
4. **Achieve Complete Understanding**: Continue discussion until there is absolute clarity on what needs to be implemented, how, the success criteria, and the impact on existing functionality

**Remember**: Code modifications are strictly prohibited until the clarification process is complete.

### Harness-specific rules

- **The arm moves nothing unless an operator has asked for that specific motion.** Diagnostics, bring-up scripts and connection checks are read-only: they may connect, read feedback and disconnect; they never enable joints, never send a motion command. Anything that enables or moves the arm is a separate, explicitly named tool. (`tools/piper_hold.py` is the first such tool: it enables and holds, never travels.)
- **Contracts before code across every boundary** (harness ↔ instrument, harness ↔ arm SDK, harness ↔ accessories): write the protocol into this spec, then implement, then version.
- **Spec adherence**: any new demand or requirement must include a corresponding update to this document. Instrument-internal facts go into the Bunny spec, not here — link, do not copy.

## Project Overview

**Platform vision (2026-09-15).** The harness is a *flexible lab automation platform*: a **baseboard** with a **robotic arm** as the default fixture, onto which the user (or we) install **modules** — the Super Bunny, labware storage, future liquid-handling or other stations — **anywhere on the baseboard**, chosen per workflow. After the hardware is bolted down, the user opens the **setup manual (web GUI)** and teaches the arm: they drag the gripper roughly to each module's **AprilTag**, the arm memorises the coarse pose, then localises every tag precisely with its **gripper camera**. From then on the harness runs workflows against the stored module poses. See "Platform Concept, Module Descriptor & Setup Workflow".

In its first configuration the Bunny Harness turns the Super Bunny from a stand-alone instrument into an automated station: the arm loads and unloads receivers (vials, well plates, vial holders) on the instrument's scale, swaps dispensers/powders, and moves labware between the instrument and storage or other stations. The user-facing goal, stated in the Bunny spec's layer architecture, is a user who writes *"cell 38: 0.2 g powder B"* and never touches hardware.

### What lives where

| Concern | Owner |
|---|---|
| Dispensing algorithm, scale, gantry, windshield, cameras, labware pose *on the instrument* | `bunny` repo (instrument) |
| Arm driver, arm calibration (base → world, TCP, hand-eye), gripper | **this repo** |
| World frame, station/slot map, labware library & placement bookkeeping, recipes, planner, scheduling | **this repo** |
| Instrument control interface (the instrument-side server) | spec'd **here** (contract), implemented in `bunny` |
| Accessories (plate/vial storage, nests, barcode, safety I/O) | **this repo** |

### Lineage and context

- Instrument: Weighing Super Bunny (1 dispenser → N receivers), itself derived from Weighing Bunny Alpha-1 and Weighing Rabbit Delta-2. See the Bunny spec "Lineage".
- The Bunny spec's section **"Labware Model, Coordinate Frames & Teaching (DECIDED 2026-09-02)"** was written for the whole harness, not just the instrument. Its decisions are adopted here unchanged and summarised under "Coordinate Frames, Labware and the Division of Labour".

## System Architecture

### Topology

```
                 ┌──────────────────────────────────────────────┐
                 │  Harness host (Python)                        │
                 │  recipe → planner → device API → drivers      │
                 └───┬───────────────────┬────────────────┬─────┘
        CAN 1 Mbps   │        TBD (Ethernet/USB, proposed TCP+JSON)     TBD
     (candleLight    │                   │                │
        USB-CAN)     ▼                   ▼                ▼
              ┌────────────┐   ┌──────────────────┐   ┌──────────────┐
              │ Agilex     │   │ Weighing Super   │   │ Accessories  │
              │ PIPER arm  │   │ Bunny (CM5 app + │   │ (storage,    │
              │ + gripper  │   │ STM32 main board)│   │ nests, I/O)  │
              └────────────┘   └──────────────────┘   └──────────────┘
```

- **Harness host** (**DECIDED 2026-09-15**): a Tokings fanless industrial mini PC — i5-7200U, 8 GB RAM, 256 GB SSD, 5× RS-232, 6× USB, 2× LAN — running **Ubuntu 26.04.1 LTS** (kernel 7.0, native SocketCAN). Hostname `bunny-harness`, user `bunny-harness`, currently on lab Wi-Fi at 192.168.4.23 (`bunny-harness.local`); `enp1s0`/`enp2s0` unused so far (planned: one to the lab LAN, one to the private cell network). Code lives in `~/bunny-harness-dev` (this spec, `tools/`, `requirements.txt`, uv venv `bunny-harness-venv` with Python 3.13.15, pyAgxArm 1.0.0 from GitHub commit `8cd90f9`, python-can 4.6.1). The candleLight adapter (serial `003B0047…`) now lives on this PC as `can0`; verified 2026-09-15 with `piper_can_check.py` (arm answering, 200 Hz). Not yet done: udev name `can_piper` + systemd-networkd auto-up (see "Development Environment"), static cell-network address, sleep masking, BIOS power-on-after-loss. The Windows 11 + WSL2 laptop remains a development client only.
- **Arm**: Agilex PIPER, 6-DOF, controlled over classic CAN 2.0 at 1 Mbps through a USB-CAN adapter. Details under "Robotic Arm".
- **Instrument**: one Weighing Super Bunny for now; the architecture must allow N instruments (the Bunny spec: "the workcell repo coordinates the arm, one or more Super Bunnies, and future stations").
- **Accessories**: **TBD**. Expected: labware storage (plate hotel / vial rack), dispenser rack for powder swaps, locating nests, an E-stop / safety circuit, possibly a barcode reader for labware identity.

### Controller inventory

| Node | Role | Link to host | Status |
|---|---|---|---|
| PIPER arm controller (`ARM_MC`, node 15) | joint servos, kinematics, motion planner (firmware) | CAN 1 Mbps via candleLight `1d50:606f` | **verified 2026-09-15** (read-only) |
| Super Bunny CM5 | instrument application (GUI, dispensing, gantry, windshield, cameras) | **TBD** — proposed TCP | not connected yet |
| Super Bunny STM32H750 | real-time motion inside the instrument | *not reachable from the harness* — only via the CM5 | — |
| Accessories | — | TBD | not chosen |

The instrument's internal CAN bus (STM32 ↔ MyActuator gear BLDC ↔ Damiao windshield BLDC) is **not** the arm's CAN bus. The harness never joins the instrument bus; the two buses stay physically separate.

## Robotic Arm (Agilex PIPER)

### Facts established 2026-09-15 (bring-up, read-only)

| Item | Value |
|---|---|
| Firmware | `S-V1.9-0` (software), `H-V1.2-1` (hardware), production date `260806`, node type `ARM_MC`, node number 15 |
| Bus | classic CAN 2.0, **1 Mbps** (sample point 0.750 as configured by the kernel; `gs_usb` clock 48 MHz) |
| Adapter | candleLight-firmware USB-CAN, USB `1d50:606f`, serial `003B00474148570A20343133`, Linux driver `gs_usb` → `can0`. (A second candleLight, serial `0039002A4759530920353131`, was bound to WSL in the past and shows as a persisted usbipd entry — two adapters exist; keep track of which is which.) |
| Feedback traffic | unsolicited at **200 Hz** per group: `0x2A1`–`0x2A8` (arm status + joint feedback), `0x251`–`0x256` (per-joint driver feedback), `0x262`/`0x263` (end-effector/gripper feedback); ≈2100 frames/s aggregate, zero bus errors over 158 k frames |
| State at check | `ctrl_mode=STANDBY`, `arm_status=NORMAL`, `motion=REACH_TARGET_POS_SUCCESSFULLY`, `teach=DISABLED`, all six joints **disabled**, no error bits |
| Pose at check | joints `[+1.329 +1.208 −0.310 −0.036 +0.689 +2.896]` rad; TCP `xyz (0.033, 0.127, 0.143) m`, `rpy (3.100, −0.063, 1.604)` rad — the folded/parked pose it was left in |

### SDK

- **`pyAgxArm` 1.0.0** (Agilex, LGPL-3.0, <https://github.com/agilexrobotics/pyAgxArm>) on **python-can 4.6.1**, installed in `bunny-harness-venv` (Python 3.13.15, a `uv` venv — it has no `pip`; install with `uv pip install --python bunny-harness-venv/bin/python <pkg>`). This is Agilex's newer SDK (supersedes `piper_sdk`, which is PyPI 0.6.2 and not installed).
- Entry points: `create_agx_arm_config(robot="piper", firmeware_version=PiperFW.V189, interface="socketcan", channel="can0")` → `AgxArmFactory.create_arm(cfg)` → `robot.connect()`. The protocol class must match the firmware: `S-V1.9-0` ≥ `S-V1.8-9` → `PiperFW.V189` (the bundled demo `pyAgxArm/demos/detect_piper_series.py` shows the two-pass pattern: connect with DEFAULT to read the firmware string, reconnect with the matching enum).
- Read API used by the check tool: `get_firmware()`, `get_arm_status()`, `get_joint_angles()` (rad), `get_tcp_pose()` / `get_flange_pose()` (m, rad), `get_joints_enable_status_list()`, `get_comm_error()`, `get_fps()`. Motion API (not used yet): `enable()/disable()`, `set_speed_percent()`, `set_motion_mode()`, `move_j`, `move_p`, `move_l`, `move_c`, `move_js`, `move_mit`; effector via `robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)`. `get_driver_version()` raises `NotImplementedError` on this driver class.
- Timestamps in feedback are host `time.time()`; each message carries its own measured `hz`.

### Arm TBDs

- Gripper: **AgxGripper for now, custom later** (decided 2026-09-15). The custom gripper must carry the gripper camera and handle SLAS plates *and* vials; chamfered self-centering jaws are preferred (Bunny spec: "mechanics beats vision").
- Mounting: base position relative to the instrument; `set_installation_pos` (horizontal/side/inverted) must match.
- Reach/payload check against the instrument's scale height and the storage positions (feasibility at plan time is mandatory — "every vendor has an analyze/simulate step").
- E-stop / safety circuit: the PIPER has no external safety input in the SDK path; how power is cut and how the software detects it is TBD.
- Deployment host and whether the arm bus needs a second adapter for a second instrument/accessory bus.

## Platform Concept, Module Descriptor & Setup Workflow (PROPOSED 2026-09-15)

Decisions taken in the 2026-09-15 discussion (operator):

| Topic | Decision |
|---|---|
| Baseboard | modules are placed **freely** (no grid, no dowels) — the software must cope with arbitrary placement |
| Arm | Agilex PIPER, fixed on the baseboard |
| Gripper | Agilex AgxGripper for now; a **custom gripper** later (carries the camera, handles SLAS plates and vials) |
| Cameras | **two**: a **gripper (wrist) camera** and a **global camera** over the baseboard. **Gripper camera first**; the global camera is a later addition |
| Module identity/pose | **AprilTag(s)** on every module; the baseboard carries its own tag(s) |
| Teaching | coarse by hand (drag-teach to the tag), fine by camera, verified before use |
| Setup UI | a **web GUI** served by the harness PC, usable on its own screen or from any browser on the network |
| Middleware | **ROS 2** (see "Software Architecture") |
| Hand-off geometry | fixed relative to the module's tag for now (modules with many internal slots, e.g. a plate hotel, are a later concern) |

### Frames

```
world ──(baseboard tag bundle, measured once per install)──► arm_base
arm_base ──(PIPER kinematics / feedback)──► flange ──(TCP calibration, per tool)──► tcp
flange ──(hand-eye calibration, per camera mount)──► gripper_cam
world ──(fine localisation of module tags, per placement)──► module_<name>
module_<name> ──(module descriptor, static)──► handoff points (nests, approach poses, safe heights)
```

- Every stored pose is in **`world`**, never in joint angles or steps — the rule inherited from the Bunny spec. Replacing or recalibrating the arm invalidates only `world→arm_base`.
- The coarse teach additionally stores the **joint configuration** the user left the arm in (elbow side, wrist flip), so fine localisation and later approaches plan from the configuration the user chose, not through the module.
- Three calibrations, three lifetimes: `world→arm_base` per install; `flange→gripper_cam` per camera mount (factory / after a crash); `flange→tcp` per tool.

### Module descriptor (PROPOSED, YAML; one file per module *type*, shipped with the module)

```yaml
module: super_bunny
version: 1
tags:                          # AprilTag bundle in the module frame (mm, module origin = tag 0 centre)
  family: tag36h11
  size_mm: 50
  layout:
    - {id: 101, xyz: [0, 0, 0],     rpy_deg: [0, 0, 0]}
    - {id: 102, xyz: [120, 0, 0],   rpy_deg: [0, 0, 0]}
footprint_mm: [420, 380, 600]  # collision box, module frame
handoffs:                      # every place the arm interacts with, in the module frame
  scale_nest:
    pose_mm_deg: [210, 190, 85, 0, 0, 0]
    approach: {direction: [0, 0, 1], distance_mm: 60}   # straight-down approach from 60 mm above
    labware: slas_footprint                              # what fits here
    capture_range_mm: 2.0                                # what the nest's chamfer absorbs
interface:                     # how the harness talks to the module's own controller
  kind: tcp_json
  discovery: mdns             # or a fixed address
requires: {shield: open, gantry: parked}                # interlocks before the arm may enter
```

The descriptor is the **L1/L2 boundary**: the user positions hardware, the descriptor carries geometry and interlocks, the transform math stays ours. `jsonschema` validates the file on load; a module without a valid descriptor cannot be taught.

### Setup workflow (the "setup manual" in the web GUI)

1. **Install hardware.** User bolts modules to the baseboard anywhere, connects power and network. No measuring.
2. **Base calibration** (per install, or "re-run if the arm was moved"): the arm images the baseboard tag bundle from several poses; result `world→arm_base`.
3. **Per module — coarse teach.** GUI lists the modules found on the network (or lets the user pick a descriptor). Arm goes limp (drag-teach), user brings the gripper camera to within ~150–300 mm of the module's tag, facing it, presses **Capture**. Stored: joint configuration + TCP pose. The GUI shows the live camera image with the tag detection overlaid so the user knows it is in view *before* capturing.
4. **Per module — fine localisation** (automatic): arm enables, moves to 2–3 viewpoints around the coarse pose (translations of a few cm, small tilts), detects the tag bundle in each, fuses the detections into one `world→module` pose with a residual.
5. **Verification** (automatic, mandatory): the arm drives the camera to a *second* feature predicted from the descriptor (the second tag, or the nest) and checks the prediction against what it sees. Residual above threshold ⇒ the module is marked **not ready** and the GUI asks the user to re-teach; it never proceeds "probably right".
6. **Feasibility** (automatic): with all modules localised, every hand-off pose in every descriptor is checked for reach (with gripper + payload) and collision against every other module's footprint. Failures are reported per module with a suggestion ("move storage 5 cm away from the arm"). Free placement is only free within reach and without shadowing — this step is what makes it safe to promise.
7. **Ready.** Poses and calibrations saved with timestamps; the workflow page unlocks.

Re-teach triggers: a module is physically moved (the user says so, or a run-time verification fails), the arm is moved (base calibration), the gripper/camera is changed (hand-eye / TCP).

### Why AprilTags are enough here — and their limits

- Wrist-camera tag pose at 150–300 mm gives **±1–2 mm / ±1°** with a 50 mm tag under decent light. That equals the spec's transport budget: *the arm transports at ±1–2 mm, the nest locates at ~0.05 mm, and the module reads the nest — never the arm.* Vision only has to land the gripper inside each hand-off's `capture_range_mm`; mechanical lead-ins on the module do the rest.
- **Single small tags have an orientation ambiguity** (the tilt sign flips under noise). Mitigation is designed in: tags ≥ 40–50 mm, a **bundle** of ≥ 2 tags per module, and **multiple viewpoints** fused. The spec forbids single-tag single-view localisation of a module.
- Lighting remains the deciding factor (Bunny spec: "controlled lighting — the whole ballgame"). Tags tolerate ambient light far better than well-plate lattices, but glare on a glossy tag kills detection: **matte-printed tags**, and a small ring light on the gripper camera if needed.
- The global camera, when it arrives, gives a coarse map of everything at once (which module is where, has anything moved) and can replace the manual coarse teach; the wrist camera remains the fine instrument.

### Web GUI (PROPOSED)

- Served **by the harness PC** (a ROS 2 node with an embedded HTTP + WebSocket server, e.g. FastAPI/uvicorn); the browser needs no ROS. Works on the PC's own display in kiosk mode and from any browser on the lab network.
- Pages: **Setup manual** (the workflow above, with live camera + detection overlay, one module at a time), **Modules** (list, status, last teach, re-teach), **Workflow** (recipe editor/runner — later), **Diagnostics** (arm state, CAN health, logs, the jog panel — marked unsupported for users).
- Safety in the UI: enabling the arm and every motion command are explicit button presses with the arm state visible; the E-stop state, when hardware exists, is shown on every page.

### Software Architecture (ROS 2 — DECIDED 2026-09-15)

ROS 2 is adopted because the platform will keep gaining modules and each becomes a node with a typed interface; MoveIt 2 gives the collision-aware planning that free placement needs; `tf2` is the frame tree above; bags give replayable failures.

- **Distribution**: **ROS 2 Lyrical Luth** (released 2026-05-22, LTS until May 2031, Ubuntu 26.04 is its Tier-1 platform). **Verified 2026-09-15** against the official apt index for Ubuntu 26.04 (`packages.ros.org`, suite `resolute`): `ros-lyrical-ros-base`, `ros-lyrical-desktop`, `ros-lyrical-rclpy`, `ros-lyrical-tf2`, `ros-lyrical-ros2-control`, **`ros-lyrical-moveit`** (85 MoveIt packages) and **`ros-lyrical-apriltag` / `apriltag-detector` / `apriltag-draw`** all exist as binaries — no source builds needed for the planned stack.
- **Python**: ROS nodes run on the **system interpreter** (`rclpy` is built against it; Python 3.14.4 on 26.04). **Implemented 2026-09-15** as `~/bunny-harness-dev/ros-venv`, a `python3 -m venv --system-site-packages` venv: it sees rclpy and the apt-provided numpy (2.3.5) and adds pyAgxArm, python-can, scipy (pip, 1.18.1), pyyaml, jsonschema, pytest, ruff, pyrealsense2, opencv-python-headless. Run nodes with `ros-venv/bin/python`. The uv venv `bunny-harness-venv` (Python 3.13) stays only for the non-ROS bench scripts in `tools/`.
- **Workspace** (`~/bunny-harness-dev` becomes a colcon workspace):

```
bunny-harness-dev/
  PROJECT_SPEC.md
  src/
    harness_msgs/         - custom interfaces (ModuleStatus, Teach*.action, Dispense.action …)
    harness_arm/          - PIPER driver node on pyAgxArm (JointState, enable/disable, MoveJ/MoveL actions), gripper
    harness_vision/       - gripper camera node, AprilTag detection, hand-eye + tag-bundle fusion
    harness_calibration/  - base, TCP, hand-eye calibration routines and the calibration store
    harness_modules/      - descriptor loader/validator, module registry, tf broadcaster for module frames
    harness_instrument/   - Super Bunny bridge node (TCP contract ↔ ROS actions/topics)
    harness_planner/      - feasibility checks, workflow execution (MoveIt 2 when needed)
    harness_ui/           - web GUI node (HTTP/WebSocket server)
    harness_bringup/      - launch files, parameters, udev/systemd assets
  tools/                  - bench scripts (piper_can_check.py, piper_hold.py …), venv-based, no ROS
  descriptors/            - module descriptor YAML files + JSON schema
  calib/                  - calibration store (world←base, hand-eye, TCP) — per machine, backed up
```

- **`ROS_DOMAIN_ID=36`** on every machine in the cell (decided 2026-09-15; keeps the cell's graph separate from any other ROS 2 system on the lab network). **Nodes on other computers** (camera Pi, CM5) join the same graph over the private cell network with that same domain ID; the Super Bunny's own STM32 stays hidden behind the CM5 (no micro-ROS there).
- Physical transport (CAN, RS-232, USB, Ethernet) is invisible to the graph: only the driver node that owns a device knows how it is attached.

### Open TBDs (platform)

- AprilTag family/size and the per-module bundle layout standard; whether every module also carries a machine-readable descriptor pointer (tag ID → descriptor) so the GUI can auto-identify modules.
- Gripper camera model (global shutter, fixed focus, ~60–90° lens), mount on the AgxGripper for now, illumination.
- Custom gripper requirements (plates + vials, camera carriage, capture range of the jaws).
- Baseboard tag bundle design and base-calibration procedure (how many views, acceptance residual).
- Verification thresholds (mm/deg) per hand-off type; what "not ready" blocks.
- Web GUI stack and kiosk setup on the PC display; authentication on the lab network.
- ~~ROS 2 distro verification on 26.04.1; MoveIt 2 availability for it~~ (verified 2026-09-15, see Software Architecture); whether the PIPER URDF from `agx_arm_urdf` matches the H-V1.2-1 hardware.
- E-stop / safety I/O (unchanged from "Arm TBDs").

## Bunny Registration & First Cycle — Sprint Plan (2026-09-16/17, DECIDED scope)

Operator's plan of 2026-09-16, reviewed and extended (additions marked **(added)**, sprint-only simplifications **(sprint)**). Goal: register one Super Bunny on the baseboard, run a complete dispenser + vial mount/dismount cycle, then soak it for 12 h+.

### Installation stage (registration)

1. **Coarse teach.** Arm limp (disabled). User drags the gripper camera to the module's **main tag**.
2. **"Set" criterion.** The setup GUI shows the live gripper camera with the detection overlay. The tag counts as *set* when it is detected, its centre lies inside the **central square** (middle third of the image), its apparent size corresponds to a 150–300 mm standoff **(added)**, and this holds for ≥ 1 s continuously **(added — a glancing detection must not set)**. The square turns green; **Capture** stores joint configuration + TCP pose + the tag pose in the camera frame.
3. Repeat for every module. **Module setup finish** → arm enables and holds (same logic as `tools/piper_hold.py`, now via the arm node).
4. **Fine localisation.** Arm visits each stored coarse pose, takes 3 viewpoints (±30 mm translation, ±10° tilt, camera kept on the tag), fuses → `world→module` with residual; then **verification** (workflow step 5): predict a second feature (the platform tag) and look. States per module: `untaught → coarse → fine → verified | failed`.
   **(added)** All automatic motion in step 4: ≤ 20 % speed, confined to a ±50 mm box around the coarse pose; the GUI has a STOP button; abort = hold.

### Working stage (one mount/dismount cycle)

0. **Preconditions (added):** bunny heartbeat < 2 s old, shield state known, instrument not dispensing, no arm error bits.
1. Arm to the bunny **standoff pose** (the main-tag viewing pose = outside the instrument envelope).
2. `shield open`, wait for **`open` confirmed** (the shield also changes the collision footprint — the arm enters only after confirmation).
3. Platform at its **loading z** (commanded or read from the bunny node). Camera to the rough view of the **platform tag** (rough pose from the descriptor ⊕ Tz(platform z)), fine-locate it (2 viewpoints), then `mount = T_platform_tag · T_tag→mount (accurate) · Δ_device (overlay)`.
4. Place/remove the **dispenser**: approach along the hand-off's approach axis from `approach.distance_mm`, insert/extract, release/grasp with gripper feedback checked **(added: grasp present / released, else abort)**, retreat.
5. Same for the **receiving-plate tag** → vial placing point.
6. Place/remove the **receiving vial**.
7. Retreat to standoff; only then may `shield close` be issued.
   **DECIDED 2026-09-16: no staging rack.** The gripper dismounts the dispenser (or vial), holds it, and mounts it back. The cycle is *dismount → hold → mount*; the bunny is the only module this sprint.

### Digital twin (tags only, this sprint)

A 3D page in the setup GUI (browser, three.js): world axes, arm base, live TCP frame and camera frustum (FK from `/joint_states`), every known tag drawn as a square with its ID, coloured by state (untaught / coarse / fine / verified / failed), the ghost of the current motion target, the bunny platform tag moving with the reported z, shield state shown as a label plus its collision box. Data: `tf` + a `/harness/tags` topic streamed over the GUI WebSocket. No meshes / URDF rendering this sprint (see the digital-twin discussion: RViz + `agx_arm_urdf` later).

### Spatial relationships to be supplied by the operator (task IV) — how they map

| Operator item | Where it lives | Precision |
|---|---|---|
| main tag → bunny origin | `descriptors/super_bunny.yaml: tags.main` | rough (only for standoff / rough views; measured, never trusted for hand-offs) |
| origin → platform tag | `tags.platform` at `z_ref`, plus `moves_along: [0,0,1]` (platform z from the bunny node) | rough |
| platform tag → dispenser mount (x y z rx ry rz) | `handoffs.dispenser_mount: {from_tag: platform, pose_mm_deg}` | accurate |
| device-dependent delta for the mount | `calib/modules/superbunny-001.yaml: dispenser_mount.delta_mm_deg` | adjustable from the GUI (jog-and-save) |
| origin → receiving-plate tag | `tags.plate` | rough |
| plate tag → vial placing point | `handoffs.vial_place: {from_tag: plate, pose_mm_deg}` | accurate |
| device-dependent delta for the vial | overlay `vial_place.delta_mm_deg` | adjustable |

Rule: every hand-off chains from the **nearest tag** (platform tag, plate tag), never from the main tag, so only the small tags need to be accurate and the main tag may be rough. Descriptor = per module *type*; overlay = per *serial* (the "device-dependent" deltas).

**(added) Also needed from the operator:** tag family / size / IDs of the three bunny tags (are they printed, matte?); the **dispenser grasp pose** (where the gripper holds it, orientation constraint) and the **vial grasp pose**; which platform z is "loading z" and where its zero is (the bunny's Z has no homing switch wired yet — steps are relative to power-on zero — so the harness **re-locates the platform tag every cycle** and uses z only for the rough view and the twin).

### Bunny side (task V) — recommendation

The CM5 runs Raspberry Pi OS Trixie; ROS 2 Lyrical has no binaries for it, and building it there costs a day. **DECIDED 2026-09-16 (operator: "bunny ros2 on harness pc").** The bunny exposes a small **TCP newline-JSON server** (the contract below, subset: `status {shield, platform_z, dispensing, hb}`, `shield open|close`, `platform load` / `platform goto_z`) and the harness PC's **`harness_instrument` node bridges it** into the ROS graph (`/bunny/shield_state`, `/bunny/platform_z`, services `/bunny/shield_open|close`, `/bunny/platform_load`). Same result on the graph, ROS stays on one machine. **(added)** The bunny node must accept *commands*, not only expose status — the working stage needs shield open/close and the platform at loading z.

### Task list (dependency order; ⇐ = needs)

| # | Task | Notes |
|---|---|---|
| 0 **(added)** | `harness_arm` node — **DONE 2026-09-16** (see "Arm node" below) | JointState 100 Hz; services enable / disable / teach-mode / hold; actions move_j, move_p, move_l with a speed cap; gripper open/close + position feedback. pyAgxArm already offers `move_j/move_p/move_l/set_motion_mode` and the arm controller does the IK → **no MoveIt this sprint (sprint)** |
| I | Camera — **camera + tag node DONE 2026-09-16; hand-eye tool written, not yet run** | RealSense **D405** (DECIDED 2026-09-16) on the AgxGripper: 7–50 cm range, global shutter, small. **Checked 2026-09-16 on the PC:** serial 261922272472, fw 5.15.1.55; `pyrealsense2` 2.58.4 (cp314 wheel) installed in `ros-venv` together with `opencv-python-headless`; colour stream 1280×720 works and intrinsics come from the device (f≈651 px, c≈(624, 365), inverse Brown-Conrady). **The camera currently sits on a USB 2 link** (a 4-port USB 2 hub shared with the CAN adapter; the PC's USB 3 root hub is empty): only 1280×720 @ 15 fps / 640×480 @ 30 fps are offered, and a video stream shares the hub with CAN. ~~**Action:** plug the D405 directly into a USB 3 port~~ — **done 2026-09-16**: camera now on the USB 3 root hub (`usb: 3.2`), 1280×720 @ 30 fps streams (~24 fps measured over 60 frames incl. start-up); CAN adapter stays on the USB 2 hub alone. Working resolution for tags: 1280×720 (a 50 mm tag at 250 mm ≈ 130 px). Auto-exposure blows out against a window — fix exposure manually in the lab. No `ros-lyrical-realsense2-camera` binary is needed: the camera node uses `pyrealsense2` directly. Bracket, USB 3 to the PC, `pyrealsense2` (pip wheel) or the realsense ROS driver; colour stream only for tags; intrinsics from the device. **(added) Hand-eye calibration** `flange→camera` (~15–20 poses on a fixed tag, `cv2.calibrateHandEye`) and **TCP** offset → `calib/`. **(sprint)** `world := arm_base` (no baseboard bundle yet) |
| II | Installation logic 1–4 | `harness_vision` (detection, set criterion, fusion) + `harness_modules` (registry, states, tf) + `harness_calibration` (store) ⇐ 0, I |
| III | Installation GUI 1–4 + tags twin | `harness_ui` ⇐ II |
| IV | Spatial relationships | operator → `descriptors/super_bunny.yaml`, overlay |
| V | Bunny TCP server (bunny repo) + `harness_instrument` bridge | contract first (below), then both sides |
| VI | Cycle sequencer (`harness_planner`) | working stage 0–7 with interlocks; first cycles with the operator watching, hand on `--release` / E-stop ⇐ all above |
| VII | Soak ≥ 12 h | log per cycle: residuals, mount/dismount success, gripper feedback, cycle time, shield/platform timings. **Stop conditions:** any drop, verification residual over threshold, arm error bits, heartbeat loss, CAN feedback loss. **Pass:** 0 drops, 0 unplanned stops, residual drift < 0.5 mm over the run. Inspect fingers and tags afterwards |

### Arm node (`harness_arm`, IMPLEMENTED 2026-09-16; **kinematic correction layer added 2026-09-18**)

`src/harness_msgs` (ArmStatus, GripperStatus, SetGripper, SetSpeed, MoveJ, MovePose) and `src/harness_arm` (`piper_node`, params in `config/piper.yaml`, `launch/piper.launch.py`). The node is the **sole owner of `can0`**; run exactly one instance (a second instance doubles `/joint_states` and both answer the same actions — seen once during bring-up).

| Interface | Type | Behaviour |
|---|---|---|
| `/joint_states` | JointState, 100 Hz | joint1..6 rad + `gripper` width m |
| `/arm/tcp_pose` | PoseStamped, frame `arm_base` | **corrected** flange pose = harness FK (SDK MDH table + `calib/kinematics.yaml` joint zero offsets, j5 −7.5°) of the reported joints |
| `/arm/tcp_pose_raw` | PoseStamped | the controller's own flange report (wrong by ~10 mm; debugging only) |
| `/arm/status` | ArmStatus | ctrl mode, enable list **from joint feedback**, error bits, teach flag, feedback age/Hz, `fault` |
| `/arm/gripper_status` | GripperStatus | width, force, driver enable, driver errors |
| `/arm/enable` `/arm/disable` `/arm/hold` | Trigger | disable = limp = drag-teach; hold = enable + move_j(current) with a 1 s drift watch |
| `/arm/set_speed` | SetSpeed | capped by `max_speed_percent` (30 this sprint) |
| `/arm/gripper` | SetGripper | width + force; returns reached / **stalled** (object in the jaws) / timeout |
| `/arm/move_j` | MoveJ action | refused if errors, teach mode, busy, or any joint jump > `max_joint_delta_rad` (1.2 rad) |
| `/arm/move_pose` | MovePose action | target in the corrected model; **IK solved in the node** (numerical, seeded from the current joints ⇒ same wrist/elbow branch, limits as hard bounds) then `move_j`; `linear=true` = IK'd waypoints every 20 mm / 0.1 rad; refused before moving if unreachable, branch-changing, outside the workspace box or inside the floor margin; fails fast (`stall_s` 2 s) if the controller clamps a joint |
| `/arm/set_joints` `/arm/gripper_release` `/arm/gripper_enable` | SetJoints / Trigger | power or free selected joints; free / re-power the gripper motor only |

Safety in the node: watchdog (`watchdog_s` 0.25) and any error bit → goal cancelled + `fault` latched, **arm stays enabled** (changed 2026-09-17 after the incident below; it used to auto-disable); cancel or timeout → re-target current joints (freeze); one motion at a time; the SDK's `disable()` return value is ignored (it returned False on a successful disable). **Incident during the bench test:** a test script fed an empty joint list, the ROS CLI padded it to six zeros, and the node drove the arm to the all-zero pose (10 % speed, nothing hit). The `max_joint_delta_rad` rail was added in response: one goal may not ask any joint to jump more than 1.2 rad; longer moves go through via points. Bench test (`ros2 launch harness_arm piper.launch.py`, then CLI calls): 100 Hz feedback, hold, wrist ±0.1 rad, jump refusal, gripper +5 mm and back — all pass.

Running ROS nodes: `.bashrc` now prepends `ros-venv/lib/python3.14/site-packages` to `PYTHONPATH`, so `ros2 run` / `ros2 launch` (system Python 3.14) see pyAgxArm, pyrealsense2, scipy, cv2. Non-interactive shells must source `/opt/ros/lyrical/setup.bash`, set that `PYTHONPATH`, `ROS_DOMAIN_ID=36`, and source `install/setup.bash` themselves (`.bashrc` exits early for them). `bunny-harness-venv/` and `ros-venv/` carry `COLCON_IGNORE` so colcon does not crawl them.

### Vision and teach tools (IMPLEMENTED 2026-09-16, camera-tested; tag pipeline awaits a real tag in view)

- **`harness_vision/tag_node`** owns the D405: `/camera/tags` (TagArray: id, pixel centre/corners, size px, camera-frame pose, distance, reprojection error, `in_center_square`), `/camera/image/compressed` (10 Hz JPEG with the overlay: central square turns green when a tag is *set*), `/camera/camera_info`. Detector: OpenCV `aruco` with the AprilTag 36h11 dictionary, one threshold scale + subpixel refinement (**27–30 fps at 1280×720** on the i5-7200U; the default 3-scale setting gave 7 fps on a cluttered scene). Pose: corners deprojected through the device's own inverse-Brown-Conrady model, then `solvePnP(IPPE_SQUARE)` on normalised rays. Params in `config/camera.yaml`: `tag_size_m` (default 0.05) with per-id overrides, `center_fraction` 1/3, size gate 50–260 px, optional fixed `exposure_us`.
- **`harness_vision/geom.py`**: 4×4 transform helpers; the PIPER's `[x y z roll pitch yaw]` is **fixed-axis XYZ (R = Rz·Ry·Rx) — verified 2026-09-16** by `handeye --check-convention` (re-command current pose: no motion; +10° about flange z: reported vs predicted 0.00 mm / 0.03°). The same check **fails at the all-zero pose**: the controller answers `TARGET_POS_EXCEEDS_LIMIT` to Cartesian targets there (wrist singularity). Rule: never issue `move_pose` from the zero pose; leave it with `move_j`. The arm node now aborts a goal when the controller reports anything but NORMAL for 3 samples.
- **`harness_calibration/handeye`** (eye-in-hand, `cv2.calibrateHandEye`): start = arm holding with the fixed tag in view at 150–250 mm; ~21 viewpoints (±30–40 mm base-frame translations, ±12–15° flange-frame tilts, ±20° rolls, combinations); 20 detections averaged per pose after a 2.5 s settle + stillness check; all four OpenCV methods solved, position-only refinement with a fitted tag scale; writes `calib/handeye.yaml` (+ raw pairs JSON). Since 2026-09-18 the arm poses it records are the corrected ones, so it can be re-run as-is; the joint-offset identification itself lives in `calib/handeye_backups/fit_kinematics_2026-09-18.py` (joint-space samples + hand-eye + offsets solved together) and is the tool to use when the arm's zero is suspected again.
- **`harness_calibration/teach`** = stage II: release (operator supports the arm, presses Enter) → live terminal line (`dx dy size dist SET t`) → SET held `--hold-s` 1.0 s → automatic `/arm/hold` → coarse record (joints, flange pose, camera-frame tag pose) → fine pass (hold view + 2 offset views ±25 mm with counter-tilt 8°, chained base→flange→camera→tag, fused, spread = residual) → `move_j` back to the hold joints → `calib/teach/<module>.yaml` (`state: coarse|fine`). Fine pass is skipped with a warning if `calib/handeye.yaml` is missing.
- **`harness_calibration/client.py`**: synchronous client (latest status/joints/tcp/tags, `trigger`, `gripper`, `move_pose`, `move_j`) for procedural scripts — the sequencer will use it too.
- Ops: `tools/node_ctl.sh arm|camera start|stop|restart|status` on the PC (detached, logs in `~/piper.log`, `~/tag_node.log`); `rosenv.sh` in the workspace root sources everything for non-interactive shells. Wi-Fi to the PC showed a 30 s outage and 120 ms latency during this session — the wired cell network remains a prerequisite for the soak.

### Hand-eye calibration log — **current result (plate #2, 2026-09-18): 1.60 mm rms / 0.73°, accepted, with the corrected kinematic model**

`calib/handeye.yaml`: flange→camera **(−64.8, 11.5, 76.9) mm, rpy (0.69°, 0.46°, −91.33°)**, fitted jointly with the joint zero offsets from 52 poses on wall tag 8 (34.7 mm): position rms 1.60 mm, max 2.85 mm, orientation rms 0.73°, max 2.7°; hold-out (fit one half, predict the other) 2.0–2.4 mm. The value is meaningful **only together with `calib/kinematics.yaml`** (the arm node applies it). Camera optical axis ≈ flange z (2° off), image-up = flange −x, camera body 56 mm off the flange axis on that side. Remaining error is dominated by PnP orientation noise of a 135 px tag (±0.5°); a 50–60 mm tag or a longer standoff would lower it. History below (plate #1) is kept for the record; every earlier number carries the j5 zero error (that is what the "k ≈ 0.8–0.92 tag scale" was).

#### Plate #1 history (2026-09-17, superseded)


**Runs 2+3 on the flat table tag 7** (169 px at ~155 mm, joint 5 parked at 60°, motions ≤ 25 mm / 8° tilts / 20° rolls at the operator's request): 16 + 13 usable pairs, merged 29. Closed-form spread ~5 mm / 1.5°; position-only refinement **rms 2.8 mm, max 4.4 mm**, tag scale k = 0.918 in both runs independently (black square ≈ **32 mm**, now the configured size for id 7). Per-pose residuals are spread over all motion types (no single bad viewpoint) → a systematic floor, most likely the accuracy of the pose the PIPER controller reports (its own nominal FK), not the camera. Both runs agree on `flange→camera ≈ (−54, 10, 48) mm, rpy (7.8°, −3.3°, −92°)` to within 1 mm. **Adopted as provisional** (`calib/handeye.yaml`, `accepted: false`): sufficient for stage II coarse/fine teach and for the first cycle *because the nest's capture range and the per-device overlay deltas absorb a few mm*; to be redone with an independent accuracy check (camera intrinsics calibration, FK check with the tag) before the soak.

**Run 4 (operator request: 20 more captures, smaller moves, longer dwell — 15 mm / 6° tilts / 15° rolls, 2.5 s settle, 30 frames per pose, tag size 32 mm):** **rms 1.76 mm, max 2.58 mm**, k = 0.993 (the 32 mm size is confirmed by the fit), `flange→camera = (−47.5, 10.0, 45.4) mm, rpy (8.6°, −1.8°, −93.6°)`. The jump from 3 mm to 1.8 mm came from the longer settle: the arm is still creeping ~1 s after the controller reports the target reached — **rule: dwell ≥ 2 s before any measurement that pairs an arm pose with an image**. This run is the adopted `calib/handeye.yaml`. Do not merge runs recorded with different assumed tag sizes (the merge of runs 2–4 gave 6 mm for that reason). Remaining ~1.8 mm is the arm's pose-reporting floor until a kinematic check is done.

Earlier attempt on the inclined Bunny tag 5 (kept for the record):

- Lab tags are **AprilTag 16h5** (not 36h11): table test tag = id 7, Bunny front tag = id 5, another tag under the windshield glass. `tag_node` family switched to `tag16h5`, **`error_correction_rate: 0`** (with OpenCV's default 0.6 the 16h5 decoder flipped id 7 ↔ 5 between views).
- Tag sizes: the operator's "50 mm" is the printed sticker; the **black square** that the pose solver uses is smaller — table tag 7 ≈ 34.7 mm (depth-measured, fronto-parallel, ±0.1 mm); Bunny tag 5 ≈ **28 mm by scale fit** (depth gave 24 mm but the tag is oblique, so that number is biased). **Calipers on the black square are required** for every tag; the descriptor must carry the black-square size.
- Joint limits: **j5 clamps at ±70.0°** (datasheet says 75°); a hand-dragged arm can sit beyond it, after which the controller answers every Cartesian target with `TARGET_POS_EXCEEDS_LIMIT`. `client.JOINT_LIMITS` + a precheck in `handeye`; `jog` moves a joint back. The hold service's drift guard raised to 0.1 rad (joints settle by ~0.06 rad when the motors take over from limp at an extended pose); plain `/arm/enable` also holds.
- New tools: `jog` (`--joint n --to/--delta`, `--print`), `center --tag-id` (2 probe moves → image Jacobian → tag centred within 30 px; also the future rough-approach step), `handeye --resolve <pairs.json>` (offline re-solve), position-only refinement with a fitted tag scale k.
- Run 1 on tag 5 (oblique, 87 px, 269 mm, 14 usable pairs; 5 viewpoints refused at the j5 limit, tilts one-sided): closed-form spread 34.6 mm / 2.4° (tag orientation of a small oblique tag is unreliable); position-only refinement with scale: **rms 5.2 mm, max 7.8 mm, k = 0.564** → NOT accepted; `calib/handeye.yaml` holds this provisional value (`accepted: false`). Next: a **flat tag facing the camera at ~200 mm with ≥ 150 px edges and a caliper-measured size**, two-sided tilts (start pose with j5 margin), then re-run.

### INCIDENT 2026-09-17 ~02:50 — arm released mid-motion by the feedback watchdog

While the camera was being repositioned toward tag 7 (arm extended, camera looking forward after a wrong kinematic guess), CAN feedback from the arm stopped for **18 s** (`ip -s link` afterwards: 61 166 error-passive transitions, no gs_usb/USB events in dmesg, bus clean again afterwards at ~2 300 frames/s). The arm node's watchdog reacted as designed at the time — **`disable()` on feedback loss** — and the arm went limp while extended and folded onto its rest stops (joints ended at j2 = −1.9°, j3 = +1.9°). Root cause of the CAN outage not yet known (candidates: arm controller reset triggered by the pose command, or a strained CAN cable at that extension — the operator was present).

**Policy change (implemented immediately):** the node **never disables the arm on its own**. Feedback loss or error bits → cancel the running goal, latch `fault`, refuse new goals until `/arm/enable` clears it; the arm keeps holding its last target by itself (the PIPER holds when CAN goes quiet). Releasing is only ever an explicit operator command (`/arm/disable`) or a future hardware E-stop. Rationale: a limp arm falls; a held arm carrying a vial does not.

Also learned: **do not guess PIPER joint-space reconfigurations** (j3's sign is the opposite of the naive planar model — j2 = 50°, j3 = −10°, j5 = 55° pointed the camera at the horizon, not the table). Reposition the camera with Cartesian translations from a known-good configuration, or through the URDF/FK once loaded.

### Floor calibration & clearance guard (IMPLEMENTED 2026-09-17)

`ros2 run harness_calibration floor`: with the arm limp the operator drags the gripper **tip** over the table; the tool samples flange poses (every 5 mm of motion) and fits the table plane `z = a·x + b·y + c` in the base frame together with the tip offset `L` along flange z (observable because the wrist orientation varied 40°). Result 2026-09-17: 181 samples over 23 × 23 cm, **table tilted 2.3° relative to the arm base**, tip offset **109.2 mm**, residual rms 2.2 / max 4.8 mm (the arm's pose-reporting floor again) → `calib/floor.yaml`. The arm node now refuses any `move_pose` goal whose *tip* would come within `floor_margin_m` of that plane (15 mm = 10 mm requested + 5 mm fit uncertainty), in addition to the flange workspace box (`z ≥ 60 mm`). Joint moves are not filtered (they only return to operator-taught configurations). The 2.3° table tilt means "world" must eventually be the baseboard tag bundle, not the arm base — a 2.3° error over 400 mm is 16 mm.

### Stage II status (2026-09-17 afternoon) — Bunny registered: coarse + fine DONE

- **Teach CLI redesigned** at the operator's request: `teach --drag-recog` (arm limp; the harness does not know which module comes — the tag id seen is looked up in `descriptors/*.yaml` → module/role; each SET tag is recorded once to `calib/teach/<module>.yaml` and one RECORDED line is printed; Ctrl-C = "module setup finish" → arm enables and holds), `teach --fine` (all coarse-taught modules: back to the taught joints, 3 views, fused base→tag), `teach --list`. Joint-limit guard in the live line (withholds SET within 8° of a limit). Tag-to-module registry: `harness_calibration/registry.py` over `descriptors/`.
- **Bunny main tag 5 registered:** coarse teach 13:25 (tag at 98 mm, 179 px); fine pass **base→tag = (391.5, 76.6, 27.8) mm, rpy (85.7°, −1.8°, −87.6°), spread 2.5 mm / 0.44° over 3 views → state `fine`**.
- The first fine passes failed at 6 mm: the descriptor size for tag 5 was wrong (28 mm). The fine pass now **self-checks the tag size** (fits the scale that makes the views agree): k = 1.256 → **35 mm**; with 35 mm in the descriptor the re-run gives k = 1.001. Rule: a wrong size shows up as the tag "following" the camera by (k−1)× the camera motion. Both lab tags are therefore ≈ 32–35 mm black squares of the same print; calipers still pending. Fine acceptance set to **3 mm** (the hand-eye floor is ~2 mm); "verification thresholds" TBD stays open.
- Fine-pass views are always `--lift` 15 mm above the taught pose (operator: the gripper was almost touching the table at the taught pose); the tip-clearance guard (floor calibration) protects the rest.

### 2026-09-18 — tag sizes settled by a range-scale test; platform tag located roughly; gripper lead

- **Tag black-square sizes** (calipers still pending, but now measured two independent ways): **tag 5 (Bunny main) = 35.8 mm** by the range-scale test (`/tmp/rangetest.py`, to become `harness_calibration rangetest`: back the camera off 0–120 mm along its optical axis in 30 mm steps; PnP range grew 0.771 mm/mm with 27.6 mm assumed ⇒ ×1.297); **tag 7 (table) = 34.7 mm** by stereo depth square-on. Same 35 mm print. A 10 mm 16h5 sticker has a 7.5 mm black square (6/8 of the white). Lessons: (1) the multi-view "scale self-check" over 20–25 mm baselines is unreliable (dominated by the ~2 mm hand-eye error), now informational only; (2) `size_px·depth/fx` is only valid square-on — replaced by `S_assumed·depth/PnP_range` in `TagDetection.depth_size_m`; (3) D405 depth is unreliable below ~100 mm and ~4 % short of PnP at 120–240 mm; (4) never merge hand-eye runs recorded with different assumed sizes. The 60 mm placement error of tag 5 seen from the first rough view (250 mm, 56° oblique, image edge) remains unexplained and is a known-unknown for oblique far views.
- `tag_node` now streams **depth** aligned to colour and publishes `depth_m` / `depth_size_m` per tag (~26 fps).
- **Gripper lead**: with the camera on the side of the AgxGripper, the **tip is 51–56 mm ahead of the camera** along the optical axis and ~58 mm to the side. `locate` therefore plans the standoff from the **tip** (`--tip-clearance` 40 mm ⇒ camera 91 mm), re-approaches once from what it sees (descriptor errors of ±100 mm are tolerated), retreats 80 mm when done, and refuses results from < 40 px detections, < 3 views, or > 3 mm spread. Rule: **the tip, not the camera, is what must clear the module.**
- **Platform tag 3** (on the dispenser head, faces the same way as the main tag, tilted ~21°): first rough view found it **31 mm right, 65 mm BEHIND the main-tag face, 252 mm up** in the module frame (operator estimate was −30, −55, 200: y sign was wrong). Localisation **FAILED** (7.5 mm tag: 36–56 px, views disagree by 6.6 mm, depth useless at 86 mm). **Blocked on a larger platform tag (≥ 15 mm, ideally 20 mm).** Descriptor `xyz_mm: [31, 65, 252]`, `rough_mm: 30`.
- `teach --fine` returns to a taught configuration via the taught flange pose first (pose move, then a small joint snap) — a direct joint move from far away is refused by the 1.2 rad jump guard. Note: `move_pose` has no jump guard (IK may choose any configuration) — an open hole, see TBDs.
- Arm node parameter `workspace_min` z restored to −0.05: the floor is guarded by the tip-clearance check, not the flange box.

### Platform tag localised (2026-09-18, 5-view cross)

`locate` now takes a **five-view cross** (centre, ±3° about the camera's horizontal axis, ±3° about its vertical axis) and averages each opposite pair, so the tilt-linked bias of a small tag cancels; the uncertainty is the spread between the centre and the two pair-means, the single-view spread is kept as a diagnostic. Result for the 7.5 mm platform tag 3 at 91 mm (47 px): **base→tag (476.1, 68.6, 234.1) mm, uncertainty 1.1 mm (per axis 0.1 / 0.3 / 1.8), single-view spread 9.5 mm** — accepted. Module-frame offset from the main tag: **(17, 56, 212) mm** (x right, y into the Bunny, z up); the operator's estimate (−30, −55, 200) had the y sign wrong. Two earlier three-view runs of the same tag disagreed with this by up to 18 mm in z, so the number is provisional until a ≥ 15 mm tag confirms it. `locate` predicts from the last measurement when one exists (`--from-descriptor` overrides), retreats 80 mm when done; the size self-check is informational only (degenerate without translation baselines).

### INCIDENT 2026-09-18 ~01:30 — arm dropped by an operator-command misread; joint 5 driver fault

The operator asked to "release the gripper, stop powering it"; the assistant called the ARM disable service. The arm went limp from the platform-view pose and fell to the right (the operator caught it; final pose flange (274, −120, 28) mm). **Rule (memory + spec): "release" is ambiguous — gripper and arm are separate motors; gripper release = `/arm/gripper_release` (new, `disable_gripper()`), arm release only on an explicit "arm".** On re-enabling, five joints came back but **joint 5 refused**: its driver had latched `driver_overcurrent + stall_status + driver_error_status` (10.8 A, 10.3 Nm, from the wrist being loaded in the fall) while the arm-level `err_status` showed nothing. `clear_joint_error(5)` then `enable()` recovered it. The arm node now (a) reads every joint's driver flags each cycle and lists them in `ArmStatus.errors` as `j5:stall_status` etc., (b) clears latched driver faults before enabling and names the joints that still refuse. Gripper facts: the AgxGripper with the current fingers opens to **32.4 mm** maximum (SDK assumes 70 mm); `/arm/gripper_release` / `/arm/gripper_enable` services added.

### 2026-09-18 evening — hardware redesign; calibrations invalidated

Operator findings: (1) the D405 body blocks the gripper (camera sits in front of the finger plane, collides with the dispenser head); the camera connection plate will be redesigned. (2) The gripper will be redesigned (fingers/opening). (3) The arm's motion during the day nudged the Super Bunny on the table, so its registration is stale.

| Item | Status | Why |
|---|---|---|
| `calib/handeye.yaml` | REDO after the new plate | camera pose vs flange changes |
| `calib/teach/super_bunny.yaml` (coarse joints, fine main tag, `tags.platform`) | REDO | Bunny moved |
| `calib/poses.yaml` platform_view | stale | Bunny moved |
| `tcp_offset_m` (floor guard), gripper max opening | REDO after the new gripper | tip length / jaw geometry change |
| `calib/floor.yaml` plane (a, b, c) | still valid | base frame; table and arm base did not move |
| tag sizes (7: 34.7, 5: 35.8, 3: 7.5 mm), descriptors, code | still valid | |

Redesign guidance handed to the operator: fingertips must be the most distal point (camera behind the finger plane, above the flange axis, tilted down so the tip is near the bottom edge of the image and ≥ 100 mm from the lens along the optical axis — D405 depth is unreliable closer); camera mount and fingers keyed/dowelled so remounting does not force a new hand-eye; Bunny clamped or bolted to the table.

Software consequence (TODO, working stage): a **registration check** before each cycle — look at the main tag from the taught view, compare with the stored base→tag pose, refuse to run if it moved by more than a few mm and ask for re-registration.

### 2026-09-18 evening — hand-eye redo #2 started; wrist-branch flip found; Cartesian joint-jump guard added

New camera plate and gripper installed by the operator. Wall calibration tag is **ID 8** (16h5, registered at 34.7 mm like tag 7; the hand-eye scale fit will check it). The camera node had to be restarted: it does not recover from a USB unplug (`wait_for_frames cannot be called before start()` forever) — TODO auto-reconnect.

The operator's dragged start posture had the wrist wound up: j4 −100° (limit ±106°), j5 64.5° (limit ±70°), j6 119.6° (limit ±120°). `joints_outside_limits` refused; j6 was jogged −30° (tag stayed in view). A 15 mm base-frame `move_pose` then made the controller **switch IK branch** (wrist flip: j4 +180°, j5 negated, j6 +180°) instead of refusing. **Finding:** in the flipped branch the controller reports the *same* flange pose (0.4 mm / 0.1°), but the camera sees the tag 85 mm displaced (tag centre 770 px → 364 px); reversing the flip along the same joint path brought it back to 770 px exactly. So the reported flange pose is not the physical flange pose across wrist branches (controller kinematic model vs the real wrist — to be characterised; both hand-eye runs so far lived in one branch, which is why they were self-consistent). **Rule: never mix wrist branches; stay in the one the registration was taught in.** Fix: `pose_max_joint_delta_rad` (0.9 rad) — `/arm/move_pose` now freezes the arm (`_stop_here`) as soon as any joint has run more than that from its start value and reports "joint-jump guard ... wrist flip". The pre-existing move_pose guard hole is closed.

### 2026-09-18 evening — ROOT CAUSE of the hand-eye residuals: joint-5 zero offset −7.5° (arm kinematic model ≠ real arm)

Hand-eye run from a wrist-relaxed posture (found by a posture optimiser using the SDK's `piper` MDH table, which reproduces the controller's reported flange pose to 0.38 mm rms over the 181 floor samples) gave 7.5 mm rms — NOT accepted. Raw-pair analysis: 30 mm base-x/y translations matched the camera to 2–4 %, but vertical (base-z) moves showed 25 % extra tag motion and flange-x tilts showed 2–3° less camera rotation than reported. Single-joint moves of j4/j5/j6 (±12–16°, hysteresis sequence) match the camera within 0.5° with no lost motion, and a joint move to the same angles reproduces a Cartesian move's camera view exactly — so the arm is deterministic and the **controller's kinematic model disagrees with the physical arm for combined wrist motions**.

Identification: 52 samples (23 random joint-space viewpoints + 9 joint-space mini-run + 20 Cartesian-run pairs with joints recovered by IK), fitted jointly for flange→cam (6), tag pose (6) and joint zero offsets j2..j5 (j1, j6 unobservable):

| model | pos rms | max | orient rms | implied tag size |
|---|---|---|---|---|
| hand-eye only | 11.4 mm | 21.5 | 2.31° | — |
| hand-eye + scale | 9.1 mm | 15.9 | 2.29° | 28.9 mm (the old "28 vs 35" confusion) |
| hand-eye + joint offsets | **1.60 mm** | 2.85 | 0.73° | — |
| hand-eye + offsets + scale | 1.60 mm | 2.82 | 0.73° | **35.0 mm** (tag really is ≈34.7) |

Offsets j2..j5 = (−1.9, +2.0, +0.5, **−7.5**)°; j5 alone gives 2.04 mm rms (−7.46°). Hold-out: fit on joint-space samples → 2.0 mm on the Cartesian pairs and vice-versa 2.4 mm. Files: `calib/kinematics.yaml` (offsets), `calib/handeye.yaml` (flange→cam (−64.8, 11.5, 76.9) mm, rpy (0.69, 0.46, −91.33)°, accepted, **valid only with the offsets applied**), `calib/handeye_backups/` (samples + fit script). This also explains the wrist-branch observation (a j5 zero error mirrors differently in the two branches) and the ≈2 mm "pose-reporting floor" seen before.

Consequences (decision pending): either (A) the arm node owns the kinematic model (MDH + offsets): publishes a corrected `/arm/tcp_pose`, and `/arm/move_pose` does branch-preserving numerical IK seeded from the current joints then `move_j` — no controller Cartesian moves at all (also removes the IK-branch flip risk); or (B) re-zero joint 5 in the AgileX firmware and re-identify. Floor plane must be re-fitted with the corrected model (offline from the saved joints in `calib/floor.yaml`). Bunny registration is stale anyway (moved) and will be redone with the corrected model.

### 2026-09-18 night — correction layer IMPLEMENTED in the arm node (`harness_arm/kinematics.py`)

The arm node now owns the kinematic model: SDK MDH table + joint zero offsets from `calib/kinematics.yaml` (param `kinematics_file`). `/arm/tcp_pose` is the **corrected** flange pose (harness FK of the reported joints); the controller's own report is on `/arm/tcp_pose_raw` (they differ by ~9 mm at the calibration posture). `/arm/move_pose` no longer sends Cartesian targets to the controller: it solves IK itself (numerical, seeded from the current joints ⇒ stays in the current wrist/elbow branch, joint limits as hard bounds, `ik_limit_margin_rad`), refuses unreachable or branch-changing targets before moving, then drives `move_j`. `linear=true` = chain of IK'd waypoints every `linear_step_m`/`linear_step_rad` (all checked against floor/workspace first). New fail-fast: a move whose joints stop short of the target for `stall_s` (2 s) aborts with "stalled short of the target" — the controller silently clamps at its limits. **Measured controller limits: j4 ±100°** (table said 106°; the operator's dragged posture sat exactly at −100.1°), j5 ±70°.

Acceptance against the camera (tag 8, corrected model): base-z +30 mm → camera 31.0 mm (was 37 mm); linear −25 mm x → 25.1 mm; linear +30 mm z with 15° roll → rotation 15.13° vs 14.95° commanded; return-to-start repeatability 0.4 mm. Floor plane refitted offline from the saved floor joints with the corrected model: `[-0.03150, -0.01323, 0.03070]`, rms 1.9 mm, **30 mm higher** than the old-model plane at the sampled area (the old plane was wrong by the j5 error, which the 15 mm margin did not cover — another reason the gripper "wanted to go lower"). `tcp_offset_m` still refers to the OLD gripper; the new gripper's tip length is not measured yet.

### 2026-09-18 night — floor re-calibrated with the new gripper and the corrected model

Floor drag (arm limp, tip on the table): 490 samples over 226 × 343 mm with 83° of wrist orientation spread. Fit: plane `[-0.00478, -0.00242, 0.02068]`, **table tilt 0.31°** (the old-model fit said 2.3° — the j5 zero error again), **new gripper tip offset 99.4 mm** along flange z (fitted, old gripper was 109.2), residual rms 1.51 mm / max 4.4 mm, no outliers (the first in-air seconds produced no off-plane samples). `config/piper.yaml` updated (`floor_margin_m` stays 0.015 = 10 mm requested + fit uncertainty). Floor guard is now consistent with the corrected pose the node publishes.

### 2026-09-18 night — Bunny re-registered (corrected model, plate #2): main tag FINE

Coarse pose taken from the held pose facing tag 5 (no drag), then a wrist-relaxed posture computed with the calibrated model (predicted tag pixel (639, 365) vs measured (641, 340) — the model is right). Fine pass, 5 views (±25 mm, 8° counter-tilt, 15 mm lift): **base→tag 5 = (417.5, 54.8, 66.9) mm, rpy (89.9°, −1.1°, −90.9°)**, spread 2.59 mm / 0.60° → FINE. (Old-model value was (400.7, 76.5, 27.8) — the Bunny was moved AND the model changed, so the numbers are not comparable.) Size self-check k = 0.948 (informational, short baseline).

### 2026-09-18 night — platform tag localised (corrected model): 0.5 mm

`locate` from the descriptor first saw nothing: the old-model offset put the tag 45 mm off, in the image corner. One centring move found it (59 px at 80 mm); the single-view position was stored as a rough prior and the 5-view cross run from it. **Result: base→tag 3 = (477.1, 81.8, 241.2) mm, rpy (86.8°, −1.1°, −92.6°), uncertainty 0.48 mm (per axis 0.3 / 0.8 / 0.5 mm), 0.17°, prediction error 3.5 mm → FINE.** Module-frame offset main→platform = **(-24.6, 58.7, 174.9) mm** (descriptor updated, `rough_mm: 3`); the old-model value (17, 56, 212) was ~40 mm wrong in x and z. Node change on the way: point-to-point `move_pose` targets whose IK needs more than `max_joint_delta_rad` are now split into joint-space via points (each floor/workspace-checked) up to `pose_total_jump_rad` 2.5 rad; the 223 mm approach had been refused otherwise.

Registration state for the cycle: main tag FINE (2.6 mm spread), platform tag FINE (0.5 mm), hand-eye 1.6 mm, floor 1.5 mm, all in the corrected model. Missing for the first cycle: dispenser mount definition relative to the platform tag (task IV numbers), the grasp, and the receiving-plate tag.

### 2026-09-18 late — gripper: fingers reversed (mapping inverted in the node), longer fingers, floor redone

- The operator mounted the fingers the other way round: motor "open" = jaws closed. Arm node params `gripper_inverted: true`, `gripper_travel_m: 0.0967` (raw reading with the jaws closed), `gripper_scale: 1.0` (jaw gap per raw metre — **not calibrated yet**, needs one caliper measurement of the full-open gap). `/arm/gripper` and `GripperStatus.width_m` are now in jaw-gap terms (close = 0). Grip test on the dispenser head: stalled at 26.8 mm (raw scale), reported force 2.1 N with a 5 N request.
- The fingers were then found too short and replaced by longer ones (finger set #3). Floor drag #2: 269 samples, 189 × 315 mm, but only 32° of wrist orientation spread. Alone it gives tip 113.9 mm and a table 7.7 mm higher / 0.8° more tilted than drag #1 — the table did not move, so tip length and plane are confounded in that fit. **Joint fit of both drags (one plane, two tip lengths): plane `[-0.00681, -0.00263, 0.02209]` (tilt 0.42°), finger set #2 = 98.1 mm, finger set #3 = 127.2 mm**, residual rms 1.5 / 1.9 mm (max 9 mm on drag #2). Config uses the joint plane and 127.2 mm (the longer, safer value); a ruler check of flange face → fingertip is requested from the operator. Rule for future drags: vary the wrist by 60°+ or the tip length is not trustworthy.

### 2026-09-18 late — camera shifted during the finger swap; hand-eye redone; kinematic offsets refined with both sessions

After the finger swap the platform tag read 3 mm off, and at the afternoon's exact wall-posture joints tag 8 sat 65 px lower in the image: **the camera had moved on its mount** (old hand-eye misfit the new samples by 17 mm). The D405 USB also dropped during the 165° joint-4 rotation on the way to the wall (cable tug; camera node restarted — it still cannot reconnect by itself; **the cable needs strain relief**). Move to the wall was planned as fold → turn → wrist → extend with every 2° of the path checked by FK for fingertip/flange/camera clearance (table 256 mm, wall 103 mm, Bunny 44 mm).

24 new joint-space samples. Fitted alone with the afternoon's offsets fixed: 2.1 mm rms, but the wall tag came out 12 mm from the afternoon's position and free offsets drifted (j5 −4.7°): **samples around a single posture cannot separate joint offsets from camera tilt.** Fix = combined fit of both sessions (76 samples): one wall-tag pose, one set of joint offsets, two camera mounts:

| model | all rms | session A | session B |
|---|---|---|---|
| no offsets | 10.7 mm | 8.8–18.8 | 8.9 |
| j5 only (−7.52°) | 2.28 mm | 1.4–2.2 | 2.7 |
| j2..j5 (−1.46, +0.81, +0.33, **−7.39**)° | 2.19 mm | 1.3–1.9 | 2.9 |

Adopted: `calib/kinematics.yaml` = the 4-offset combined values; `calib/handeye.yaml` = mount B, flange→cam **(−58.3, 12.1, 79.7) mm, rpy (−1.73°, −0.45°, −91.15°)** (camera moved ≈ 8 mm / 2.8° vs mount A); wall tag 8 at (177.0, −435.0, 349.3) mm in base — **a fixed reference for a quick hand-eye check in future**. Session B residual (2.9 mm) is higher than A's (1.8): the mount may not be fully rigid — operator to check the plate screws. Floor refitted with the combined offsets (both drags jointly): **table level to 0.01°**, 19.1 mm above the base origin, finger set #2 = 101.0 mm, **finger set #3 = 130.9 mm**, rms 1.5 / 1.8 mm — the level table is an independent confirmation of the offsets. Bunny registration to be redone with this model.

### 2026-09-18 night — hand-eye from MANY postures: j5 correction revised to −5.35°; accuracy floor ≈ 4 mm across postures

Operator request: same wall tag, more arm postures. Reachability scan (FK/IK offline, joint margins, wall/table clearance for fingertips, gripper base and camera): 17 of 144 candidate views reachable, almost all from one side. 48 samples collected in one go over 6 clearly different postures (camera roll −135°…+90°, wrist pitch of both signs, base −104°…−21°, range 0.20–0.28 m), plus the 24 single-posture samples from earlier tonight.

Findings: (1) holding j5 at −7.4° (the single-posture value) misfits the 7-posture data by **8.6 mm**; free, it goes to **−5.35°** with 2.6 mm rms. The −7.4° was an artefact: from one posture, camera tilt and j5 trade off. (2) Richer arm geometry (link twists, lengths) does not generalise: predicting a posture left out of the fit stays at 7–9 mm with all data, **4.0 mm** within tonight's one-go set. (3) Giving the earlier single-posture group its own camera position only improves 3.15 → 2.36 mm, so the mount moved little tonight. (4) Independent check on the two floor drags (759 samples): every corrected model fits at 1.6–1.8 mm vs 3.7 mm uncorrected; the floor data alone prefer j5 ≈ −6.5° with a flat minimum from −5 to −7.5°.

Adopted: `calib/kinematics.yaml` = **j5 −5.35° only** (j2..j4 not significant); `calib/handeye.yaml` = gripper base→camera **(−64.0, 9.3, 79.3) mm, rpy (0.71°, −0.66°, −90.68°)**, rms 2.58 mm / 0.59°, wall tag 8 at (176.3, −440.6, 342.4) mm. Floor refit: plane `[-0.01148, -0.00511, 0.00418]` (tilt 0.72°), finger set #3 = **144 mm** (131 mm under the −7.4° model → a ruler on gripper base→fingertip discriminates the two models). **Working conclusion: absolute accuracy across different arm postures is ≈ 4 mm (arm + mount, not reducible by modelling with this tag); repeatability in one posture is 0.3 mm. Sub-millimetre work must therefore be LOCAL: paired-view measurement of a tag near the work point, and motions defined relative to that tag from a similar posture.** Vocabulary decided by the operator: "gripper base" (not "flange").

### 2026-09-18 night — operator's two-posture test: wrist axis angle found; model now 4 parameters

Test proposed by the operator: **same gripper base pose (per the model), reached by two different postures, camera looking at the wall tag — the two pictures should be identical.** For the PIPER only two such postures exist within the joint limits (wrist turned either way: j4 ±180°, j5 mirrored, j6 ±180°; needs j4 ≈ ±90°, j6 ≈ ±90°); a random search over joint space found 4073 candidate pairs with the tag in view and clear of wall/table; used A = (−44.5, 44.9, −43.7, −89.7, 30.6, −93.9)°, B = (−44.5, 44.9, −43.7, 90.3, −19.9, 86.1)°.

Run 1: A→B 9.2 mm; back to A 2.9 mm off (one-off camera/cable shift). Run 2 with the operator watching the cable (A, B, A, B, A): **repeatability 0.1–0.3 mm, A↔B = 8.08 / 8.06 mm** → a systematic model error. Retuning j5 alone cannot remove it. Fit of 48 multi-posture samples + the 5 two-posture measurements: **θ4 −0.22°, θ5 −5.38°, α5 −0.52° (joint-4 and joint-5 axes are not exactly perpendicular), α6 −0.12°** → two-posture mismatch **8.1 → 1.3 mm**, unseen-posture prediction **3.5 → 2.1 mm**, fit rms 1.56 mm. Adding link lengths or shoulder/elbow zeros does not improve the unseen-posture score → not adopted. `harness_arm/kinematics.py` now takes `alpha_offsets_rad` as well (`PiperKinematics.from_file`), `calib/kinematics.yaml` carries both, the arm node loads both. `calib/handeye.yaml`: gripper base→camera (−63.8, 9.3, 83.5) mm, rpy (0.41°, −0.31°, −90.75°); wall tag 8 at (180.0, −443.3, 343.5) mm. Floor refit: plane `[-0.01221, -0.00446, 0.00453]`, finger set #3 = 144.1 mm, rms 1.74 mm. **The two-posture test (3 min at the wall) is the standard health check for camera mount + arm model from now on.**

### 2026-09-19 — wall tag raised to 0.86 m; base-turned posture pairs; model confirmed, floor ≈ 2.6 mm

Offline search showed the same-gripper-base-pose test can also be done with the **base turned half a turn and the shoulder swung over the top**, but only for a tag about **0.85 m above the table** (0 of 24 views at 0.65 m, 3–5 at 0.85 m, 1–2 at 0.95 m); "facing the base" is irrelevant. Operator raised tag 8 to (28, −449, 864) mm. 31 views with such pairs found; 4 pairs run (A, 2 perturbed, B, 2 perturbed, A), paths checked every 2° for elbow, wrist, gripper base, fingertip and camera against wall, table and a box around the Bunny. A-repeat 0.3–0.7 mm (one 2.5 mm cable event); **A vs B 2.8–7.6 mm**.

Fit of all 80 samples (6-posture set, wrist-turned pair, 4 base-turned pairs), scored by predicting each group when left out: current 4 wrist parameters **2.65 mm**; adding shoulder/elbow zeros 2.69; axis twists 2.68–2.76; link lengths 2.96; gravity sag of shoulder/elbow 2.82; joint angle scale errors 2.66–2.88. **Nothing beats the 4-parameter model**, whose values reproduced (θ5 −5.30°, θ4 −0.23°, α5 −0.54°, α6 −0.10°). Saved: `calib/kinematics.yaml`, `calib/handeye.yaml` (80-sample fit; the reference wall tag is now the RAISED position). Remaining ≈ 2.6 mm across very different postures = tag measurement noise at 220–300 mm / 55° obliquity (jitter up to 1.2 mm on an 89 px tag), cable/mount hysteresis (0.3–2.5 mm seen), and unmodelled arm effects. A larger tag is the next lever, not more model terms.

### 2026-09-19 — DECISION: keep the PIPER, design around its absolute accuracy

Operator decision after the calibration campaign: the arm stays. Accuracy budget to design with: **same posture repeated 0.3 mm; same gripper base pose with the wrist turned the other way 1.3 mm; with the base turned half a turn 3–7 mm; postures not in the fit 2.6 mm on average.** The pair-3 camera shift (2.5 mm, cable unattended) did not persist into pair 4 (refit with a separate camera position: 2.14 → 2.11 mm), calibration left as saved. **Design rule for every handoff: look at the tag next to the work point from close by, then move only a short distance in a similar posture (relies on repeatability, not absolute accuracy). Never use a position measured in one posture from a very different posture.** Cheap improvements, not blocking: clip the camera USB cable to the arm (strain relief); re-zero joint 5 with AgileX's procedure (5.3° is damage/bad zero, not tolerance); print a larger calibration target (50–60 mm tag or a ChArUco board). Open: re-measure the Bunny main tag with the 80-sample calibration; `teach --fine` should get the paired-view design of `locate`. Vocabulary fixed by the operator: "gripper base" (not flange), "posture" (one set of six joint angles), "spread" (scatter between views), "rms" (fit error only).

### Risks named up front

- **Gripper fingers**: stock AgxGripper fingers may not hold the dispenser or a vial — finger adapters are a hardware task, not software.
- RealSense driver on Ubuntu 26.04 / Python 3.14 (fallback: `pyrealsense2` wheel in `ros-venv`, or OpenCV over the UVC colour stream).
- PIPER `move_p` orientation convention (verify against `get_tcp_pose` before trusting it).
- Tag glare / lighting; the CAN adapter hang seen on the laptop (a 12 h soak needs the PC-side adapter and the bunny heartbeats to be stable — the persistent CAN config on the PC is a prerequisite).

### Open questions (answer before task I/IV)

1. ~~RealSense model~~ — DECIDED: **D405**.
2. ~~Staging rack~~ — DECIDED: none; hold and re-mount.
3. Tag family / size / IDs — to be read from a photo of the bunny's tags (operator will provide images).
4. "Set" = green square + Capture button (assumed) or auto-capture — **deferred to task III** (operator: discuss when at that stage).
5. Platform z: operator will add Z homing when needed; until then the harness re-locates the platform tag every cycle.
6. Fingers for the dispenser and the vial — **deferred to task VI**.

## Instrument Control Interface (harness ↔ Super Bunny) — PROPOSED (superseded 2026-09-27 by the Bunny's `harness` mode, see the Bunny spec "Harness Control Interface")

The Bunny spec lists "System ↔ robotic arm: TBD (owned by the system-level integration repo)". This section is that contract. Nothing below is implemented on either side.

- **Transport (proposed)**: TCP, newline-delimited JSON, one connection per instrument, instrument is the server (the CM5 already runs a TCP server for the camera Pi — `camera_pi/camtop_server.py` — so the pattern exists in `bunny`). Ethernet preferred; USB-gadget Ethernet acceptable on the bench.
- **Every request carries a `v` (protocol version) and an `id`; every reply echoes `id` and carries `ok` plus either `result` or `error {code, message}`.** Long operations reply immediately with `accepted` and later emit `done`/`failed` events on the same connection; the harness polls `status` as a fallback.
- **Verbs (proposed, first set)**:

| Verb | Meaning | Instrument side |
|---|---|---|
| `status` | health, heartbeat state, windshield state, gantry parked?, scale reading, dispensing state, active powder | composes existing modules |
| `shield open` / `shield close` | windshield motion; **the harness may only approach the scale with `shield == open` confirmed** | `windshield.py` |
| `gantry park` | move gantry to the loading-safe pose and hold | `gantry.py` |
| `load begin {labware_id, definition}` / `load end` | harness announces it is about to place/remove a receiver; instrument tares/verifies after `load end` | new |
| `locate` | camera pose fit of the declared labware; returns `{x_A1, y_A1, theta, residual}` or `needs_teach` | `labware_locate.py` |
| `dispense {well, powder_id, mass_mg}` | one well; blocking-with-events | `dispensing/core.py` |
| `weight` / `tare` | scale access | `scale_link.py` |
| `abort` | stop everything, close gate, leave hardware safe | existing stop paths |

- **Safety interlocks are stated in the contract**: the instrument refuses `dispense` while `load` is open; the harness refuses arm motion into the instrument envelope unless `status` reports `shield=open` and `gantry=parked` within the last 2 s. Heartbeats both ways, 1 s period, 2 s timeout — same numbers the instrument already uses for CM5↔STM32.
- Decisions still needed before implementation: authentication (none on a private cell network?), discovery (static IP per instrument vs mDNS), whether `dispense` returns per-well actual mass and the tap log, event vs poll for progress, and error code namespace.

## Coordinate Frames, Labware and the Division of Labour

Adopted from the Bunny spec (DECIDED 2026-09-02) — read that section for the reasoning; only the harness-side consequences are listed here.

- **Layer architecture**: L4 Recipe → L3 Planner → L2 Device API → L1 Calibration → L0 Motion. The harness owns L2–L4 across devices and the arm's own L0/L1. Users may add at any layer; **nobody but us redefines L1**.
- **World frame**: labware pose is stored in a machine-independent world frame, never in gantry steps or arm joint angles. Three calibrations with three lifetimes: labware pose (per placement), robot base → world (per install, per robot), TCP (per tool, redone on tool swap or crash). Camera-on-arm, if ever, adds hand-eye (`cv2.calibrateHandEye`).
- **Division of labour**: **the arm transports at ±1–2 mm, the nest locates at ~0.05 mm, and dispensing reads the nest — never the arm.** Arms are repeatable, not accurate; do not use the PIPER as a metrology device. Consequence for this repo: every place the arm puts labware gets a chamfered lead-in nest, and plate/vial hand-off design starts with the gripper's capture range, not with vision.
- **Labware definitions**: Opentrons Labware Schema v2 (PROPOSED in the Bunny spec), extended under `bunnyExtensions`. A vial is a 1-well definition. The harness holds the library; the instrument receives the definition with `load begin`.
- **Well naming**: recipes use `A1`/`C6` names, never bare integers (column-major vs row-major ambiguity). Must be frozen before any recipe format is.
- **A1 orientation** comes from a placement convention marked on the deck/nest, never from the well grid.

## Safety (draft)

1. Read-only by default (see Prompt Guidelines). Enabling joints is an explicit operator action, logged.
2. Speed: bring-up and all unattended motion at a low `set_speed_percent` (value TBD); full speed only after the workspace envelope and interlocks below exist.
3. Workspace envelope: a software keep-out volume around the instrument (scale, gantry travel, windshield sweep) enforced at the planner; the instrument-side interlocks (`shield=open`, `gantry=parked`) are checked before every entry.
4. Loss of instrument heartbeat or arm feedback → arm stops (`disable` or hold, TBD which is safer for a gripped plate) and the run aborts.
5. Hardware E-stop cutting arm motor power: TBD (see Arm TBDs). Until it exists, a person stays at the arm power switch during any motion.
6. Gripped-object policy on fault: never open the gripper on a fault above the deck; hold and alert.

## Operational Workflow (draft — mirrors the instrument's cycle, sequencing TBD)

*Superseded for the first cycle by "Bunny Registration & First Cycle — Sprint Plan" above.*

1. **Plan**: recipe resolves wells, powders, receivers; feasibility (reach, capacity, labware compatibility) is checked before anything moves.
2. **Load receiver**: `shield open` + `gantry park` confirmed → arm picks the receiver from storage → places it in the scale nest → `load end` → instrument tares → `locate` (camera) or teach fallback.
3. **Swap dispenser/powder** when needed: arm exchanges the dispenser on the gantry platform; instrument clamps and runs its seating check (instrument-internal).
4. **Dispense** per well via `dispense`; harness records per-well results and labels.
5. **Unload**: `shield open` → arm removes the receiver → storage/next station. Repeat.

## Repository Structure (PROPOSED — superseded 2026-09-15 by the ROS 2 workspace layout in "Software Architecture"; kept for the current pre-ROS state)

```
Bunny Harness/code            (this directory; not yet a git repository)
  PROJECT_SPEC.md             - this document (authoritative)
  tools/
    piper_can_check.py        - read-only PIPER link check (verified 2026-09-15)
    piper_hold.py             - enable + hold at current pose / --release / --status (first enable 2026-09-15)
    can_up_wsl.sh             - WSL2: attach adapter, load driver, bring can0 up @1 Mbps
  harness/                    - (to be created, contracts first)
    arm/                      - PIPER driver wrapper + calibration (L0/L1 for the arm)
    instrument/               - Super Bunny TCP client (the contract above)
    labware/                  - library (Opentrons v2 JSON) + placement bookkeeping
    frames/                   - world frame, transforms, calibration store
    planner/                  - recipe → plan, feasibility
    cli/                      - operator tools (jog, teach, run)
  bunny-harness-venv/         - uv venv, Python 3.13 (untracked)
```

`~/bunny-harness-dev` is a symlink to this directory; `~/bunny-dev` is a symlink to the instrument repo checkout. The harness code must **not** import from the instrument repo — the instrument is reached only through its network interface.

## Development Environment

### Harness PC (Ubuntu 26.04.1) — target host

SSH: `ssh bunny-harness@192.168.4.23` (laptop alias `harness-pc`, key auth installed 2026-09-15). Setup done: openssh-server, can-utils, git, uv 0.12.15, Python 3.13 via uv, venv + `requirements.txt` (pyAgxArm is **not on PyPI** — it installs from GitHub, pinned by commit). Bring `can0` up until the persistent config below is installed:

```bash
sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up
cd ~/bunny-harness-dev && bunny-harness-venv/bin/python tools/piper_can_check.py --channel can0
```

Planned persistent config (not yet applied): udev rule naming the adapter `can_piper` by USB serial, and a systemd-networkd `.network` unit with `[CAN] BitRate=1M RestartSec=100ms` so the interface is up at boot and recovers from bus-off on its own.

### Development laptop: Windows 11 + WSL2 (`Ubuntu-20.04`)

Kernel `6.18.33.2-microsoft-standard-WSL2` ships SocketCAN (`can`, `can_raw`, `vcan`, `gs_usb`, `slcan`, … as modules). `can-utils` (`candump`, `cansend`) is installed. Two constraints shape the procedure:

1. The USB-CAN adapter is a Windows device; it reaches WSL through **usbipd-win**. `usbipd bind` needs one UAC-elevated run per adapter (persists across reboots); `usbipd attach --wsl` is needed after every replug/reboot.
2. **`sudo` in this distro requires a password**, so root steps in automated sessions go through `wsl.exe -d Ubuntu-20.04 -u root -- <cmd>` (WSL grants root without a password). Kernel modules are shared by all WSL2 distros, so loading them from any distro works.

Verified procedure (2026-09-15), packaged as `tools/can_up_wsl.sh`:

```bash
# Windows side (PowerShell, admin once):    usbipd bind --busid 2-4
usbipd.exe attach --wsl --busid 2-4                       # after each replug/reboot
wsl.exe -d Ubuntu-20.04 -u root -- sh -c 'modprobe gs_usb && modprobe can_raw'
wsl.exe -d Ubuntu-20.04 -u root -- sh -c 'ip link set can0 type can bitrate 1000000 && ip link set can0 up'
ip -details -statistics link show can0                    # state ERROR-ACTIVE, bitrate 1000000
candump can0                                              # 0x2A1..0x2A8, 0x251..0x256, 0x262/0x263 streaming
bunny-harness-venv/bin/python tools/piper_can_check.py    # firmware / status / joints, read-only
```

Failure signatures: `ip link` shows no `can0` → adapter not attached or `gs_usb` not loaded (`lsusb | grep 1d50:606f`, `lsmod | grep gs_usb`); `can0` up but `candump` silent → arm unpowered, or CAN_H/CAN_L swapped/unplugged (the Bunny spec's lesson: a swapped pair is silent *and* harmless-looking — the bus shows zero errors); `bus-off`/error counters climbing → bitrate mismatch on some node.

`can0` up, `candump` silent **and** a `cansend` probe shows TX packets 0 with no error counters → the adapter itself is hung (seen after the Windows GUI, see Bring-up Log); replug it.

The busid (`2-4` today) is whatever `usbipd list` shows for `1d50:606f`; the script resolves it.

### Python

`bunny-harness-venv` (uv 0.12.5, CPython 3.13.15): `pyAgxArm 1.0.0`, `python-can 4.6.1`, `typing_extensions`, `wrapt`, `packaging`. No pip inside — use `uv pip install --python bunny-harness-venv/bin/python …`. A `requirements.txt`/`pyproject.toml` is to be added with the first package.

## Bring-up Log

- **2026-09-15 — ROS 2 Lyrical Luth installed on the harness PC.** From the official apt source (`ros2-apt-source` 1.3.0, suite `resolute`): `ros-lyrical-desktop` 0.13.0, `ros-lyrical-moveit` 2.15.0, `ros-lyrical-ros2-control` 6.9.0, `ros-lyrical-apriltag-detector` 3.0.4, ros-dev-tools, colcon, rosdep (initialised), vcstool. `.bashrc` sources `/opt/ros/lyrical/setup.bash`, sets `ROS_DOMAIN_ID=36`, and sources the workspace overlay. Smoke test passed (C++ talker → Python listener). Workspace `~/bunny-harness-dev` created (`src/`, `descriptors/`, `calib/`, `.gitignore`); first `colcon build --symlink-install` succeeded with 0 packages. `ros-venv` created (see Software Architecture). Disk after install: 17 GB used of 233 GB.
- **2026-09-15 — first enable: hold-in-place verified.** `tools/piper_hold.py` captured the pose (joints `[+1.632 +1.642 −0.732 +0.047 +0.724 +2.715]` rad, TCP `xyz (−0.018, 0.247, 0.142) m`), set speed 10 %, enabled all six joints (0.15 s), sent a zero-length `move_j` to the captured angles; the arm went `STANDBY → CAN_CTRL`, worst drift over the 5 s watch 0.0095 rad (sag on joints 2/3 as the position loop took the load), no error bits. Left enabled and holding. Release: `piper_hold.py --release`.
- **2026-09-15 — adapter hang after the Windows GUI.** The ArmRobotUA host software (Agilex `cando.dll` driver) was used on the same candleLight adapter, and afterwards Linux could not use it: `can0` came up but no frame was received or transmitted, `ip link set down` timed out on the USB control path, and after a usbipd detach Windows enumerated it as *Unknown USB Device (Device Descriptor Request Failed)*. A pnputil device restart did not help; **only a physical replug** did. Rule: after any Windows-side use of the adapter, replug it before handing it back to WSL.
- **2026-09-15 — PIPER CAN link verified (read-only).** candleLight adapter bound and attached to WSL2, `gs_usb` loaded, `can0` up at 1 Mbps, bus `ERROR-ACTIVE`, 158 846 frames received with 0 errors / 0 dropped in the check window, 3 frames sent (the SDK's firmware/config requests). `pyAgxArm` connected, firmware `S-V1.9-0` / `H-V1.2-1`, status NORMAL / STANDBY, no error bits, all joints disabled, feedback at 200 Hz. No enable, no motion. Tooling landed: `tools/piper_can_check.py`, `tools/can_up_wsl.sh`.

## Open TBDs (consolidated)

See also "Open TBDs (platform)" above.

- Deployment host and cell network layout (static IPs? one switch?).
- Harness ↔ instrument protocol details (see the PROPOSED contract) and its implementation on the CM5.
- Gripper choice and labware hand-off geometry; nests on the scale pan (must ride on the balance and be tared — instrument-side question) and in storage.
- Arm mounting position, installation orientation, base → world calibration method (touch-off on nest features vs camera).
- Safety circuit / E-stop and the fault policy for a gripped object.
- Labware library format freeze (Opentrons v2 + `bunnyExtensions`) and well-naming convention.
- Recipe format (L4) — only after the two items above.
- Whether this directory becomes a git repository now (recommended: yes, `dev`/`main` like `bunny`) and its GitHub home/organization (the Bunny spec notes the umbrella naming is under discussion).

## Common Commands

```bash
# harness PC
source /opt/ros/lyrical/setup.bash && source ~/bunny-harness-dev/install/setup.bash   # done by .bashrc
cd ~/bunny-harness-dev && colcon build --symlink-install   # after adding a package/entry point
ros2 node list; ros2 topic list                          # what is running

# laptop (WSL2)
tools/can_up_wsl.sh                                     # WSL2 only: attach + driver + can0 up @1 Mbps
bunny-harness-venv/bin/python tools/piper_can_check.py  # read-only arm link check (exit 0 = answering)
bunny-harness-venv/bin/python tools/piper_hold.py       # ENABLES the arm: hold at current pose (supervised)
bunny-harness-venv/bin/python tools/piper_hold.py --release   # disable all joints -- support the arm first
bunny-harness-venv/bin/python tools/piper_hold.py --status    # read-only
candump can0                                            # raw bus
ip -details -statistics link show can0                  # link state + error counters
```

---

This specification is the authoritative reference for all development activities on the Bunny Harness project. Instrument-internal matters are specified in the `bunny` repository's `PROJECT_SPEC.md`.

## 2026-09-20 — DECISION: liquid dispensing belongs to the Bunny, not the harness

- The Bunny can now take a liquid dispenser head (stepper pump fed from a GL45 reagent bottle) as well as the powder head, and dispenses liquid by weight.
- The pump's stepper driver goes on the Bunny's own controller (it has spare stepper pins). No Teensy, no separate board.
- No new ROS node. The weight loop (read scale -> slow pump -> stop) runs inside the Bunny, next to the scale. The harness only sends
  "dispense X g of liquid" over the same TCP-JSON link as for powder and receives the final weight; prime / purge / installed-head queries go the same way.

## 2026-09-26 — Volume determination module moved to its own repo

The dilute-to-volume module (Z stepper + camera platform, RMD-L-4005 V-block clamp, Arducam OV9281, Teensy 4.1) has its own
spec and code in `~/dilute-to-volume-dev` (laptop) / `~/dilute-to-volume-dev` (harness PC). Its ROS 2 node `harness_volume`
runs on the harness PC and joins this workspace's ROS domain; the Teensy command set and the node interface are defined there.

## 2026-09-27 — Arm release must not free the gripper (FIXED)
`/arm/disable` (and the two fault paths in hold) called `robot.disable()` with no joint index, which frees joint 7 = the
gripper as well; an object dropped from the jaws when the arm was released. Now `_disable_joints()` frees joints 1..6 only.
`/arm/gripper_release` remains the only way to free the gripper. Also noted: `/arm/gripper_enable` re-powers at the last
COMMANDED gap, not the measured one — it moved the jaws (TODO: use the measured width). Shell shortcuts: `tools/arm hold|release|grip <mm> [N]|grip-release|status`.

## 2026-09-27 — DECISION: one control pathway (ROS 2), all modules from the harness PC — IMPLEMENTED

Every module operation is a ROS 2 service/action/topic of its node on the harness PC; the `harness` CLI
(`tools/harness` + daemon `tools/harness_cmd.py`, replaces `tools/arm`), scripts and any GUI only call those.
Nodes: `/arm` (harness_arm), `/bunny` (harness_instrument/bunny_node — TCP/JSON client of the Bunny app's
harness mode, `python3 main.py harness`, Bunny repo `python/harness_server.py`; spec section "Harness Control Interface" there), `/dtv` (harness_dtv, lives in the
dilute-to-volume repo, symlinked into `src/`). Compound operations: `harness_planner` (fill_to_mark).
New interfaces in harness_msgs: FlaskLines, MoveZ, MoveV, WeighTo, LiquidPush, Barcode, GantryJog.
`tools/harness-nodes start|stop|status` starts everything. Command ↔ module ↔ function table: **`docs/CONTROL_PATHWAY.md`**.
