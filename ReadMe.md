# Compliant controllers

Authors: [Tobit Flatscher](https://github.com/2b-t), [Alexander Mitchell](https://github.com/mitch722) (2024-25)



This package contains a **compliant controller for robotic arms in ROS Noetic** and extends on the [`gen3_compliant_controllers`](https://github.com/empriselab/gen3_compliant_controllers) repository and wraps it as a [`JointTrajectoryController`](http://wiki.ros.org/joint_trajectory_controller) that performs interpolation in between different set-points. This means that the controller can be used in combination with [MoveIt](https://moveit.ros.org/install/). As a result of wrapping the controller as JointTrajectoryController, this code base has been largely rewritten in comparison to gen3_compliant_controllers.
Currently this package only supports robotic arms with revolute and no prismatic joints.

We include a video of some of the capabilities of the controller below.


https://github.com/user-attachments/assets/e98561c3-c4f2-404f-8530-9cf1f278964a



The video above shows the joint-, task- and joint and task-space control modes. The joint and task-space mode is default. It is possible to vary the gains of the joint and task space controller so that it becomes either a task or joint space controller only using dynamic reconfigure. 

The controller can be used for pin insertion and for replanning in the event of human interaction as is shown in the video below:

https://github.com/user-attachments/assets/611a5bd9-6b1f-4e88-bd31-5ce5a897ff2d

The default task and joint-space controller is tuned for both expansive movements as well as fine assembly insertions. We show these behaviours in the video below.

https://github.com/user-attachments/assets/c386813f-6a8c-42f3-81cf-0325f8897e6d

## Usage

### Running on the hardware

This code must be used in combination with the **`kortex_hardware` hardware interface** (see [here](https://github.com/applied-ai-lab/kortex_hardware) for the installation instructions) as the hardware interface implements torque filtering as well as **gravity compensation**. The 

You will have to run the `kortex_hardware` hardware interface as well as this controller. The hardware interface launches in effort mode by default, meaning that the controller can be started.

Proceed to activate the controller by opening the `rqt_controller_manager` GUI, selecting the controller and switching it to `running`:

```bash
$ rosrun rqt_controller_manager rqt_controller_manager
```


### Running in simulation

This controller does not work well in simulation, even when increasing friction and damping for Gazebo inside the URDF. Probably the reason for this is that the `kortex_hardware` hardware interface implements torque filtering as well as gravity compensation that are both not handled by the simulated hardware interface. It seems to be workable when increasing stiffness of the system by increasing the joint stiffness matrix (e.g. to `8000`), joint compliance proportional gain matrix (e.g. to `100`) as well as the joint compliance derivative gain matrix (e.g. to `10`).

You can test the controllers on a single arm by installing `ros_kortex` (see [here](https://github.com/Kinovarobotics/ros_kortex)) and then **launching the Gazebo simulation**:

```bash
$ roslaunch kortex_gazebo spawn_kortex_robot.launch
```

Then **run the `compliant_controller`** making sure it uses the same name space as the robot, by default `my_gen3`,

```bash
$ roslaunch compliant_controllers joint_space_compliant_controller.launch robot_description_parameter:=/my_gen3/robot_description __ns:=my_gen3
```

adjust the gains if necessary (as discussed above) and **switch to the compliant controller**:

```bash
$ rosservice call /my_gen3/controller_manager/switch_controller "start_controllers: ['joint_space_compliant_controller']
stop_controllers: ['gen3_joint_trajectory_controller']
strictness: 1
start_asap: false
timeout: 0.0"
```

Finally you can control the arm with the **`rqt_joint_trajectory_controller`** graphic user interface:

```bash
$ rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller __ns:=my_gen3
```

Choose the corresponding controller manager, select the desired joint trajectory controller and switch it to active. You should now be able to move all joints independently in a smooth manner with the sliders.


## Sending Smooth Trajecotories and Replanning

As mentioned, this controller is a compliant controller wrapping a [`JointTrajectoryController`](http://wiki.ros.org/joint_trajectory_controller). You can stream a set point message or a **sparse** trajectory, which is a list of set points. When a single set point is sent to the controller, this will be the input to the control algorithm. Alternatively, the controller will interpolate between the current robot motion and the input trajectory. This is explained in more detail in the subsections below.

### Streaming Joint Trajectory Points

Streaming single [`JointTrajectoryPoint`](https://docs.ros.org/en/noetic/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html) at high frequency works well for closed-loop replanning and tele-operation. This is achieved by publishing [`JointTrajectoryPoint`](https://docs.ros.org/en/noetic/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html) to the [`JointTrajectoryController`](http://wiki.ros.org/joint_trajectory_controller) topic.

### Sending Joint Trajectories

Sending a [`trajectory of set points`](https://docs.ros.org/en/noetic/api/trajectory_msgs/html/msg/JointTrajectory.html) to the `JointTrajectoryController's`](http://wiki.ros.org/joint_trajectory_controller) action server, triggers the controller to interpolate between the robot's current motion and the new trajectory. Documentation on replanning with the trajectory follower controller is found [`here`](https://wiki.ros.org/joint_trajectory_controller/UnderstandingTrajectoryReplacement).

The controller interpolates between trajectories using splines of varying order. See the table below for more details:

| Spline Order  | Poses | Velocities | Accelerations | Notes                |
| ------------- | ----- | ---------- | ------------- | -------------------- |
| 3             | Yes   | No         | No            | Not recommended      | 
| 4             | Yes   | Yes        | No            |                      |
| 5             | Yes   | Yes        | Yes           |                      |   


The trajectory follower fits a spline starting at the *current* joint positions, velocities and accelerations and ending at the beginning of a *future* set point. As shown above, smoothest results are achieved when velocities and accelerations are included in the new trajectory. 

Furthermore, do not send a new trajectory with a set point at the current joint state. For example, if you are at timestep t, do not send a pose for this timestep, but send the set point for t + 1 as the initial or only pose. The trajectory follower will interpolate from its own states at t and find a smooth path to the set point at t + 1. See Figure 2 in [`UnderstaningJointTrajectoryReplacement`](https://wiki.ros.org/joint_trajectory_controller/UnderstandingTrajectoryReplacement) for an example of this.
   

## Code Breakdown

The algorithms and ROS control architecture are written in separate files. The non-ROS parts algorithms are found in

```bash
https://github.com/applied-ai-lab/compliant_controllers/blob/feat/joint-task-combo/include/compliant_controllers/<MODE>/controller.h
and
https://github.com/applied-ai-lab/compliant_controllers/blob/feat/joint-task-combo/src/<MODE>/controller.cpp
```
where the **MODE** is the controller type e.g. (joint_task_space, joint, task).

## Known issues

This package has a few limitations mostly stemming from its initial implementation. We will try to resolve these over time.

- The controller **time step** is **hard-coded** into the controller, setting it to an actual realistic time does not seem to work.
- Running the simulation currently requires modifications to the `ros_controllers` repository to run in **simulation**. A template argument has to be added to the `JointTrajectoryController` with the default value `HardwareInterfaceAdapter` to the `JointTrajectoryController`: `template <class SegmentImpl, class HardwareInterface, template <class HW, class S> class Adapter = HardwareInterfaceAdapter> class JointTrajectoryController` (see [here](https://github.com/ros-controls/ros_controllers/blob/678b92adfd9242c93b78c066a8369c7665ea1421/joint_trajectory_controller/include/joint_trajectory_controller/joint_trajectory_controller.h#L178))  and the joint trajectory controller in this repository has to call this `CompliantHardwareInterface` instead. Otherwise the code will run into a **segmentation fault inside Gazebo**.
- Currently dynamic reconfigure does not support variable size arrays. For this reason we implemented dynamic reconfigure for a fixed size list of parameters holding sufficient parameters for most traditional robotic arms (7). In case the robot does not have the corresponding joints, the last additional degrees of freedom will not be used.

