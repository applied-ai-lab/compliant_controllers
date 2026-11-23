# Tesollo / TORQUE commissioning — 24 November 2026

This is a runnable first-trial path, with physical commissioning still pending. Start one arm;
repeat on the other before running both. Use the robot's existing emergency-stop facilities and
the native FRANK soft-stop bridge. The existing soft-stop watchdog remains enabled.

## Prepare the workspaces

Apply the tested source snapshots/bundles described in `deployment/README.md`. On the control NUC,
build `kortex_api`, `kortex_description`, `kortex_driver`, `compliant_controllers`, `frank_description`,
`frank_moveit_config` and `frank_kortex`. On the operator PC, build `frank_gello`; the rest of the
native GELLO/model stack should already be installed from the new ROS2 repositories.

The control NUC uses the pinned ROS snapshot from `frank-ros-2-control-nuc`. Install dependencies
through rosdep with these source packages present, then build and source the workspace:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select kortex_api kortex_description kortex_driver \
  compliant_controllers frank_description frank_moveit_config frank_kortex \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF
source install/setup.bash
```

Both SDK headers and library are now **Kortex API 2.8.0**. The new machine documentation records
2.8.0 firmware on both arms; confirm the installed version when connected. No firmware upgrade
is part of this change. This native driver build targets Linux amd64.

Run long-lived launch commands in separate terminals, or start their existing Procman commands.

## Bring up POSITION hold first

On the operator PC, start the existing FRANK model with **Tesollo selected**, and the usual soft
stop. Procman may already be running these; keep the single existing model publisher. For manual
bringup the corresponding commands are:

```bash
ros2 launch frank_description description.launch.py ee:=tesollo_dg4f hardware:=real
ros2 launch frank_soft_stop soft_stop.launch.py gui:=true
```

On the control NUC, run this **instead of** another arm-driver process:

```bash
ros2 launch frank_kortex arms.launch.py ee:=tesollo_dg4f mode:=compliant sides:=left
```

This starts `left_arm_controller` in POSITION mode and loads
`left_arm_compliant_controller` **inactive**. The left controller manager is
`/left_arm_controller_manager`. Tesollo is driven separately on master-nuc through `frank_tesollo`;
the arm driver must report no internal-bus gripper and must not spawn a Robotiq controller.

Confirm the physical hand/model, joint order and measured posture. Check that `/joint_states`
contains seven finite arm positions/velocities/efforts and that the driver reports finite positive
URDF torque limits. Verify soft-stop press/release in POSITION mode first.

The default effort command signs are seven `+1` values, from Kinova's example. They are a
commissioning candidate, **not measurements on FRANK**. The driver's loaded-joint startup sign
check cannot establish the signs of lightly loaded joints. Verify signs and gravity support on
the supported arm before tracking/contact trials; configure the per-arm `left_effort_signs` or
`right_effort_signs` launch argument if measurements require a change. Changing signs requires
restarting that driver; do not infer a command sign merely from the negated feedback convention.

## Activate compliance, then engage GELLO

With the arm supported for the first transition, release the soft stop and activate the effort
controller explicitly:

```bash
ros2 control switch_controllers -c /left_arm_controller_manager --strict \
  --deactivate left_arm_controller --activate left_arm_compliant_controller
```

The driver seeds measured torque before entering TORQUE mode, performs its startup sign check,
then applies its per-cycle slew limit (0.5 N·m) and URDF absolute effort limits. The controller
starts at the measured pose with the commissioning profile. Observe gravity support and a quiet
hold before engaging a leader. Record `/joint_states` and
`/left_arm_compliant_controller/commanded_effort`; the latter is the request before driver limits.

On the operator PC:

```bash
ros2 launch frank_gello gello.launch.py ee:=tesollo_dg4f compliant:=true gui:=true
```

GELLO retains its existing calibration, engagement, limit and collision checks and starts paused.
Use the existing calibrated offsets; align the leader and engage the left arm. The new launch
flag directs targets to `/left_arm_compliant_controller/joint_trajectory` (and the corresponding
right topic). It does not change leader mapping, input rate or filtering. A right arm without
fresh follower state remains paused during the one-arm trial.

Begin with small, slow motions. Then check gentle contact, sustained contact, release from contact
and quiet hold. The profile is a starting point; TORQUE mode has different dynamics from the old
CURRENT backend. Increase gains only after observing these behaviours. Runtime changes use ROS2
parameters, for example:

```bash
ros2 param set /left_arm_compliant_controller task_k '[100.0,100.0,100.0,10.0,10.0,10.0]'
```

## Stop, manual reposition and resume

GELLO **Pause** stops new references; the compliant controller keeps holding its last reference.
The physical soft stop belongs to the driver. After soft-stop release the driver restores
POSITION hold and ignores ongoing effort commands until the controller is explicitly reactivated.
Pause GELLO, then use two switches to reset the controller:

```bash
ros2 control switch_controllers -c /left_arm_controller_manager --strict \
  --deactivate left_arm_compliant_controller --activate left_arm_controller
ros2 control switch_controllers -c /left_arm_controller_manager --strict \
  --deactivate left_arm_controller --activate left_arm_compliant_controller
```

For pendant/manual guiding, pause GELLO, deactivate the arm controller, then deactivate the hardware
component. The driver returns actuators to POSITION and the base to SINGLE_LEVEL_SERVOING, keeping
the API connection available for reactivation:

```bash
ros2 control switch_controllers -c /left_arm_controller_manager --strict \
  --deactivate left_arm_compliant_controller
ros2 control set_hardware_component_state -c /left_arm_controller_manager \
  left_kinova_arm_KortexMultiInterfaceHardware inactive
# Reposition using the robot's normal pendant/manual-guiding workflow, then:
ros2 control set_hardware_component_state -c /left_arm_controller_manager \
  left_kinova_arm_KortexMultiInterfaceHardware active
ros2 control switch_controllers -c /left_arm_controller_manager --strict \
  --activate left_arm_compliant_controller
```

Confirm the first reference/torque is reseeded at the new measured pose, align the leader again,
and explicitly re-engage. A failed mode switch or restoration must remain stopped; inspect the
driver error before retrying. Do not use `reset_fault` to bypass an engaged soft stop.

Repeat with `sides:=right`, then `sides:=both`. Robotiq remains selectable with `ee:=robotiq_2f_85`;
its controller and payload follow the existing end-effector table.

## Record on the robot

Record actual loop timing/overruns and feedback age on the control NUC, torque signs and gravity
agreement, holding and tracking, contact/release behaviour, controller switching, physical stop
and release, manual reposition/reactivation, communication fault recovery and restart. Software
tests cannot qualify these SDK/firmware/mechanical behaviours. Keep the ROS1 path available for
comparison until these checks pass.

## MoveIt to GELLO calibration

After the first supported torque/hold checks, activate both arm compliant controllers before a
both-arm calibration plan. Keep GELLO paused. Start a single MoveIt process with:

```bash
ros2 launch frank_moveit_config move_group.launch.py ee:=tesollo_dg4f mode:=compliant
```

In the operator trial Procman configuration, select `arm_mode=compliant` before starting arms,
MoveIt and GELLO; their routes then agree. The compliant controllers still start inactive. In RViz,
plan and execute the `gello_calibration` named pose through the compliant action servers. Confirm
a successful action result and quiet hold, then calibrate/align GELLO and explicitly engage.
Small MoveIt trajectories and GELLO now share the compliant controller; POSITION-first remains
an initial commissioning check. Topic targets cannot override an active action. Cancellation, path
tolerance failure and expired goal tolerance request measured-pose hold. Verify stop/release and
controller reactivation independently on the hardware.
