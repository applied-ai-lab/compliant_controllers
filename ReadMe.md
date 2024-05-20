# Compliant controllers

Author: [Tobit Flatscher](https://github.com/2b-t) (2024)



This package contains a **compliant controller for robotic arms in ROS Noetic** based on the [`gen3_compliant_controllers`](https://github.com/empriselab/gen3_compliant_controllers) repository but wrapping it as a [`JointTrajectoryController`](http://wiki.ros.org/joint_trajectory_controller) that can be used in combination with [MoveIt](https://moveit.ros.org/install/). This resulted basically in a rewrite of the code base.

You can test the controllers in simulation on a single arm in simulation by installing `ros_kortex` as described [here](https://github.com/Kinovarobotics/ros_kortex):

```bash
$ roslaunch kortex_gazebo spawn_kortex_robot.launch
```

Then run the `compliant_controller` making sure it uses the same name space as the robot, by default `my_gen3`, and switch to the compliant controller

```bash
$ roslaunch compliant_controllers compliant_controller.launch robot_description_parameter:=/my_gen3/robot_description __ns:=my_gen3
$ rosservice call /my_gen3/controller_manager/switch_controller "start_controllers: ['compliant_controller']
stop_controllers: ['gen3_joint_trajectory_controller']
strictness: 1
start_asap: false
timeout: 0.0"
```

Finally you can control the arm with the `rqt_joint_trajectory_controller` graphic user interface:

```bash
$ rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller __ns:=my_gen3
```

Choose the corresponding controller manager, select the desired joint trajectory controller and switch it to active. You should now be able to move all joints independently in a smooth manner with the sliders.



#### Known issues

The following issues should be noted by the users of this package:

- The simulation currently does not seem to work correctly (neither does it for the original code). This seems to be related to general problems simulating effort interfaces in Gazebo.
- Furthermore the code currently requires modifications to ROS control to run. A template argument has to be added to the `JointTrajectoryController` with the default value `HardwareInterfaceAdapter` to the `JointTrajectoryController`: `template <class SegmentImpl, class HardwareInterface, template <class HW, class S> class Adapter = HardwareInterfaceAdapter> class JointTrajectoryController` (see [here](https://github.com/ros-controls/ros_controllers/blob/678b92adfd9242c93b78c066a8369c7665ea1421/joint_trajectory_controller/include/joint_trajectory_controller/joint_trajectory_controller.h#L178)) .

The following improvements are scheduled to be made to this repository.

- Currently the controller parameters can't be configured with dynamic reconfigure. This is mainly due to a restriction of dynamic reconfigure that does not allow for variable-size parameters. We will implement a fixed size list of parameters that holds more parameters than necessary. In case the robot does not have the corresponding joints, the corresponding gains will be unused.
- Simulating the hardware interface in Gazebo can be tricky. We should add a simple robot that only holds few joints. This might allows us to simulate the robot better in simulation. Additionally one could play around with the friction parameters to improve this behavior.
- The actual control part currently is extracted into a single file. This would allow us to test the controller independently with unit tests. One could e.g.
  - Test that the controller controlling a robot in an upright position that should stay in that position does not generate a substantial force.
  - Similarly we could make sure that a generated effort points into the right direction and is within reasonable bounds.
- A task-space compliant controller could be created by porting the other controller from the initial repository.

