"""The module interface's adapter nodes (2026-11-20; the workflow repo's module_interface/HARNESS_PORT.md).

Each file is one device: a `railab_harness.module.Module` subclass (the contract's side) + a small LINK object that is
a ROS client of the device's EXISTING node (which keeps all its own checks). The Module never calls rclpy; the link
can be replaced by a fake (tools/module_check_<x>.py).

  arm_module     the PiPER  (link: harness_arm's piper node)       ros2 run harness_modules arm_module
  bunny_module   the Bunny  (link: the bunny node)                 ros2 run harness_modules bunny_module
  camera_module  the gripper camera (link: the camera / tag node)  ros2 run harness_modules camera_module

Instance names are the node parameter `instance` (arm_1, bunny_1, camera_1). Nothing here is in `harness nodes`.
"""
