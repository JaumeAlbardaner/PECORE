# First deliverable
Author: Jaume Albardaner i Torras

This README has been written in order to include the answers to the questions proposed by the professor.

<!-- <p align="center">
   <img src =https://github.com/JaumeAlbardaner/ninjacar_mppi/blob/master/gif/ninjacar.gif>
</p> -->
## Contents

1. [Questions and answers](#1-questions-and-answers)
2. [How to run](#2-how-to-run)
3. [Troubleshooting](#3-troubleshooting)

## 1. Questions and answers
#### What is the output of your node? topic type?

 - This package runs a total of 4 types of node (see diagram below):
    -   **tf_adder**: Node developed in the tutorials to add the 'map' frame, necessary for the execution of the rest of the deliverable.
    -   **tf_remap**: Node that publishes all the tf that require some level of computation, such as the aruco_H_camera.
    -   **static_transform_publisher**: Node that publishes static transforms. Used to publish aruco_H_desCam and desCam_H_desRob.
    -   **pbvs**: Unlike the previous nodes, this one actually publishes to a topic. It publishes a node of type [`geometry_msgs/Twist`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) to topic `/jackal_velocity_controller/cmd_vel_unstamped`. The message contains the desired velocities to perform on the Jackal robot.

#### Have you built your own position controller or used the ROS navigation stack?

 - I have built a position controller based on the Pose-Based Visual Servoing (PBVS) that was viewed in class. It includes some correction due to the vehicle not being omnidirectional.


#### How do you think Gazebo is actually driving the robot using your commands?

 - When Gazebo receives a command to drive the robot at a certain velocity, this command is passed to the controller. The controller in this case is diff_drive_controller, which takes the longitudinal X velocity and the angular Z velocity and transforms it into the wheel's individual velocities, as it can be seen in the last lines (260-265) of the `update` code:

    ``` 
    // Set wheels velocities:
    for (size_t index = 0; index < static_cast<size_t>(params_.wheels_per_side); ++index)
    {
        registered_left_wheel_handles_[index].velocity.get().set_value(velocity_left);
        registered_right_wheel_handles_[index].velocity.get().set_value(velocity_right);
    }
    ```
    The output is sent to the actual robot (in this case to its Gazebo model), which most likely will have its own inner control scheme. This control scheme is robot-specific and should not be changed if the simulation is to remain realistic.

    Information extracted from:

    * https://sir.upc.edu/projects/rostutorials/10-gazebo_control_tutorial/index.html#ros-control-overview 
    * https://control.ros.org/galactic/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html

#### Please, provide a simple scheme of the modules being used.


<p align="center">
   <img src =./img/ROS_Diagram.png width="60%">
</p>

## 2. How to run

Before running the simulation, the package needs to be built:
```
cd ~/pecore_ws && colcon build
source install/setup.bash
```

Afterwards, the appropiate launch file can be ran:
```
ros2 launch first_deliverable demo.launch.py
```

## 3. Troubleshooting

So far the only possible problem encountered is that some python libraries need to be installed.

In case such occurence happens, a ``requirements.txt`` has been included. Please try installing those packages and building the package again (preferrably, try it first on a separate conda environment):

```
pip install -r requirements.txt
```