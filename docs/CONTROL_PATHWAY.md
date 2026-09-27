# Control Pathway — all module operations from the harness PC (2026-09-27)

One pathway: ROS 2 on the harness PC. The `harness` command (`tools/harness`, linked into `~/bin`, works from any directory), scripts and any GUI only call the interfaces below; only a module's node touches its hardware. The Bunny runs no ROS: its own app in harness mode (`python3 main.py harness`, Bunny repo `python/harness_server.py`, GUI not shown) owns its UART/scale/scanner and serves TCP/JSON on port 7801 to the `/bunny` node.

Start/stop everything: `harness-nodes start | stop | status` (arm + camera nodes, dtv node, bunny node, Bunny server). `harness --help` prints the command list.

Status: **have** = works today · **new** = to write.

| CLI command | Module | ROS interface | Function | Status |
|---|---|---|---|---|
| `harness arm enable` / `release` | arm | `/arm/enable`, `/arm/disable` | power joints 1–6 on / off (gripper untouched) | have |
| `harness arm hold` | arm | `/arm/hold` | hold current posture, reply at once | have |
| `harness arm grip-release` / `grip-enable` | arm | `/arm/gripper_release`, `/arm/gripper_enable` | free / re-power gripper motor only | have |
| `harness arm grip <mm> [N]` | arm | `/arm/gripper` (SetGripper) | gripper to gap and force (≤ 20 mm near the Bunny) | have |
| `harness arm move-pose <x y z roll pitch yaw> [--joint] [--speed N]` (mm, deg) | arm | `/arm/move_pose` (action) | straight-line TCP move (`--joint` = joint-interpolated) | have |
| `harness arm move-j <j1..j6 deg> [--speed N]` | arm | `/arm/move_j` (action) | joint move | have |
| `harness arm status` | arm | `/arm/status`, `/arm/gripper_status` | joints on/off, gripper gap | have |
| `harness arm teach ...` / `locate ...` | arm + camera | `harness_calibration teach`, `locate` programs | find a module's tag, store its pose | have |
| `harness bunny status` | bunny | `/bunny/status` (String JSON) | housing, weight, pump, dispensing, powder, link | have |
| `harness bunny weight` / `tare` | bunny | `/bunny/weight` (Float64), `/bunny/tare` | scale (`scale_link.py`, `core.tare`) | have |
| `harness bunny housing open` / `close` / `release` | bunny | `/bunny/housing_open`, `/bunny/housing_close`, `/bunny/housing_release` | windshield motor (`windshield.py`) | have |
| `harness bunny weigh-to <g> [powder\|liquid] [powder_id]` | bunny | `/bunny/weigh_to` (WeighTo) | dose by weight (`core.weighing_bunny`) | have |
| `harness bunny liquid weigh <g>` | bunny | `/bunny/weigh_to` material=liquid | liquid dose by weight (`dispensing/liquid.py`) | have |
| `harness bunny liquid push <steps> [rate]` / `pull` / `suck` / `stop` / `pos` | bunny | `/bunny/liquid_push` (LiquidPush), `/bunny/liquid_suck`, `/bunny/liquid_stop`, `/bunny/status` | pump stepper in steps (`pump.py`) | have |
| `harness bunny barcode [timeout_s]` | bunny | `/bunny/barcode_read` (Barcode) | wait for the next scan, selects the powder (`barcode_scanner.py`, `powder_db.py`) | have |
| `harness bunny load begin` / `end` | bunny | `/bunny/load_begin`, `/bunny/load_end` | load handshake: weigh-to refused while a load is open | have |
| `harness bunny abort` | bunny | `/bunny/abort` | stop dispensing and the pump (housing left holding) | have |
| `harness bunny gantry park` / `jog <dx dy dz_up>` / `locate` | bunny | `/bunny/gantry_park`, `/bunny/gantry_jog`, `/bunny/locate` | gantry / top camera (`gantry.py`, `labware_locate.py`) — Super Bunny only, bunny-003 answers "no gantry" | have (bridge) |
| `harness dtv status` | dtv | `/dtv/status` (String JSON) | z, V angle/current/temp/state, fault (Teensy `STATUS` + CAN) | have |
| `harness dtv lines` / `frame [file.png]` | dtv | `/dtv/lines` (FlaskLines), `/dtv/image` | neck edges, ring row, surface rows, gap (`find_lines.py`, `flask_camera.py`) | have |
| `harness dtv z rel <mm>` / `move <mm>` / `zero` / `stop` (+ = camera DOWN) | dtv | `/dtv/z_move` (MoveZ), `/dtv/z_set_zero`, `/dtv/z_stop` | camera Z stepper (Teensy `Z REL/MOVE/SETZERO/STOP`) | have |
| `harness dtv z home` | dtv | `/dtv/z_home` | Z zero switch (not installed: answers ERR) | new (hardware) |
| `harness dtv v clamp` / `open <mm>` / `release` / `home` | dtv | `/dtv/v_clamp`, `/dtv/v_open` (MoveV), `/dtv/v_release`, `/dtv/v_home` | V-block BLDC via 0xA4 position moves, stop at \|current\| > 1.5 A (`teensy.py`) | have |
| `harness run fill-to-mark [--max-ml 5] [--offset-px 0] [--watch]` | planner | `harness_planner.fill_to_mark` over `/dtv/lines` + `/bunny/liquid_push` | dose until the surface reaches the ring, suck back; Ctrl-C stops the pump | have |
| `harness run place-head` / `pick-head` | planner | — | arm puts the dispenser head on / off the Bunny at a taught pose | new |
| `harness run weigh-receiver` | planner | — | housing open → arm place → load end → weigh-to → arm pick | new |
