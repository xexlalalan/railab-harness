"""Two simulated module nodes for the module interface (docs/module_interface.md), no hardware, no ROS.

bunny_sim.py  a Bunny on an in-memory device: the whole adapter pattern in one file
arm_sim.py    an arm on an in-memory device

Copied from the workflow repo's harness_skeleton/examples/ (2026-11-20); arm_sim's follow_path was
then changed to the real adapter's meaning (one speed per segment, the first waypoint = where the arm
is). They import the core as `railab_harness`: put src/railab_harness on sys.path first (as
tools/module_demo.py and tools/module_gateway.py --sim do).
"""
