# Combined compliant controller — ROS2 Jazzy

This branch ports FRANK's ROS1 **combined joint/task-space** controller to a native ros2_control
effort controller. The nominal-state/friction observer, quaternion task error and joint/task
spring superposition are retained. Only this controller is built; the other ROS1 variants and
adapters are historical source.

The first hardware target is a seven-axis Gen3 with Tesollo DG4F-M, **Kinova TORQUE mode**, Ubuntu
24.04 and ROS2 Jazzy. Use the accompanying `ros2_kortex/ros2-compliant`,
`frank-robot-ros-2/compliant-torque` and `frank_gello/compliant-torque` branches.
Physical commissioning is still required. Software results and dependency pins are in
[`docs/validation.md`](docs/validation.md); the hardware procedure is
[`docs/commissioning.md`](docs/commissioning.md).

## Interface

Plugin: `compliant_controllers/JointTaskSpaceJointTrajectoryController`.
It claims seven `<joint>/effort` commands and position, velocity and effort states. Commands and
feedback use **ROS joint signs and N·m**. The core subtracts ROS-sign measured effort in the
observer, replacing the old addition of raw, oppositely signed Kortex feedback.

Input: `~/joint_trajectory`, `trajectory_msgs/msg/JointTrajectory`.
All seven joint names are required; names may be reordered. Positions must be finite and inside
bounded-joint URDF limits. Optional velocity/acceleration arrays must be complete and finite;
acceleration requires velocity. Empty, partial, duplicate-name, expired and malformed trajectories
are rejected, leaving the current reference in place. Feedforward effort input is rejected.

`~/follow_joint_trajectory` provides the standard `control_msgs/action/FollowJointTrajectory`
action for MoveIt. It reports desired/actual/error feedback, completion, cancellation and path/goal
tolerance failures. Canceled or failed goals hold the measured pose; completed goals hold the final
reference. New action goals preempt old ones. Topic commands are rejected while an action is active:
pause GELLO before MoveIt execution and engage it after the action completes. Deactivation/controller
errors abort outstanding goals; inactive controllers reject goals.

Scalar position/velocity tolerances support 0 (use default) and -1 (disable). Nonzero terminal velocities, positive acceleration
tolerances, multi-DOF trajectories and component tolerances are rejected because the plugin lacks
measured acceleration/multi-DOF state. Configuration parameters `action_path_tolerance` (0.5 rad),
`action_goal_tolerance` (0.03 rad), `action_stopped_velocity_tolerance` (0.05 rad/s) and
`action_goal_time_tolerance` (2 s) provide commissioning defaults; zero disables a scalar limit.
These are independent of stiffness gains and need on-robot qualification.

Native `frank_gello` follows its current configuration: **50 Hz**, position-only, **50 ms** horizon
with the merged main defaults. The public
ROS2 trajectory sampler supplies linear interpolation for these points and quintic interpolation
when positions, velocities and accelerations are provided. Header stamp zero starts immediately.
Continuous joints use the nearest physical turn; bounded joints keep absolute coordinates.

Output diagnostics: `~/commanded_effort`, `std_msgs/msg/Float64MultiArray`, at 100 Hz, in configured
joint order. This is the controller's filtered request **before** the driver's slew/absolute limits;
it is not a measurement of torque applied by the robot.

## Dynamics and lifecycle

The controller reads the controller manager's robot description. Pinocchio reduces it to the seven
named arm joints, preserving the tool, mounting transforms and fitted hand inertia. Uncontrolled
joints are locked at the model's neutral configuration. Tesollo finger posture and estimated
inertias therefore remain a fixed-payload approximation, as in the old dynamics model.

Gravity is added **once, in the controller**. The driver sends total torque and adds no gravity.
The observer integration remains fixed at 1 ms; configuration requires a 1 kHz controller rate.
The old nominal 30 Hz effort-feedback coefficient (two updates per read) and 100 Hz output
coefficient move from `kortex_hardware` into this wrapper. The output filter now acts on torque,
before Exclaim's limits. TORQUE firmware dynamics and URDF limits differ from the old CURRENT
path, so the physical response is not asserted to be identical.

Activation seeds the reference and observer from measured state and clears old trajectories.
Reactivation after manual repositioning repeats this seeding. Non-finite feedback, model failures
or non-finite output return a controller error, allowing the manager to deactivate the controller
and the hardware to restore POSITION hold. Deactivation itself does not pretend that zero torque
is a physical stop: mode transitions belong to the driver.

`config/ros2/commissioning.yaml` is a deliberately lower-stiffness starting profile, not qualified
hardware tuning. Gain arrays can be changed atomically through ROS2 parameters. Dimensions,
finite/nonnegative entries and strictly positive rotor inertia/joint stiffness are checked before
publication to the control thread. The old `setMaxJointError` dimension typo is fixed; integral
observer gain and its error bound both start at zero to preserve disabled integral action.

## Local validation

With sibling checkouts in this repository's parent directory, start the dedicated Docker context
and build the pinned image:

```bash
docker --context colima-frank-ros2 build --platform linux/amd64 \
  -t frank-compliant:jazzy -f compliant_controllers/docker/Dockerfile.jazzy compliant_controllers/docker
docker --context colima-frank-ros2 run --rm --platform linux/amd64 \
  --mount type=bind,src="$PWD",dst=/repos \
  --mount type=volume,src=frank-compliant-validation,dst=/ws \
  frank-compliant:jazzy bash /repos/compliant_controllers/docker/validate.sh
```

Run these from the **parent directory** containing the sibling repositories. The Dockerfile pins
the ROS base digest and FRANK's 16 November 2026 ROS snapshot. amd64 is emulated on this Mac.
LTO is disabled for validation because GCC's LTO jobserver crashed under emulation; this does not
qualify robot-host timing. Regenerating the historical trace is optional:

```bash
source /opt/ros/jazzy/setup.bash
python3 /repos/compliant_controllers/test/ros2/generate_legacy_trace.py
```

The trace compiles the original ROS1 source at `e146030dc8e5bb264ccecf8efaedeb80906c10cf`
independently, with a ROS Duration/logging shim. It tests source equivalence on the same Pinocchio
version, not equivalence to an unverified deployed ROS1 binary.

## Read-only gravity monitor

`compliant_controllers/GravityMonitor` can run alongside a POSITION controller in the same arm
manager. It claims **no command interfaces** and shares position/velocity/effort states. It uses
the same model reduction and `computeGravity` function as the compliant control law. The companion
`frank_kortex` launch loads it into existing managers:

```bash
ros2 launch frank_kortex gravity_monitor.launch.py sides:=left
```

`/<side>_arm_gravity_monitor/comparison` (`control_msgs/JointTrajectoryControllerState`, 100 Hz)
contains model gravity in `reference.effort`, measured torque in `feedback.effort`, and their
difference (model minus measured) in `error.effort`. Joint names/order and measured positions and
velocities accompany each sample. `output` is empty. `/valid` and `/quasi_static` Boolean topics
at 10 Hz indicate finite calculations and a reported-velocity-based settling check. Invalid
samples are skipped without returning a controller error to the motion manager. Stopping the
launch unloads the monitor, leaving motion-controller ownership intact.

The monitor reads the selected manager's full description and locks non-arm joints at neutral,
including fingers, preserving the mounted payload. Compare at rest, without extra contact or
objects: measured torques also include dynamic/friction/closed-loop effects. This diagnostic
computes the gravity component only; it does not simulate the complete friction observer or
qualify hardware torque-command signs and closed-loop behaviour. Timing on the real NUC remains
part of commissioning.
