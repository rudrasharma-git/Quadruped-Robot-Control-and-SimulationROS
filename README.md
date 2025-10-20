# Qdog_ros2
QDog is a quadruped robot which is fully based on ROS 2 and Micro-ROS. 

### References:
- [`HyperDog` | IEEE SMC-2022](https://ieeexplore.ieee.org/document/9945526)
- [HyperDog Demo](https://youtu.be/Dx1U2J1avO0)
- [IEEE SMC-2023 presentation](https://youtu.be/mFgaS3f5-pw)
- [HyperDog Playlist](https://www.youtube.com/watch?v=1CIkmu7lIlY&list=PL8ZSjYfd0W-1BoGKr-xrBt6LwaWn_EVaN)

## Contains
This repository contains ros2 packages for quadruped robot Q-Dog.
packages are :
  1. **`Qdog_msgs`** : this package contains the msgs those used by other packages.
  
        1. `JoyCtrlCmds` : this contains the control variables of the robot from the gamepad
              - `bool[3] states` : { start, walk, side_move_mode} 
              - `uint8 gait_type` : to change the gait type
              - `geometry_msgs/Pose pose` : to control slant(x,y) and roll,pitch,yaw
              - `geometry_msgs/Vector3 gait_step` : gait_step.x = steplen_x, gait_step.y = steplen_y, gait_step.z = swing_height
              
        2. `Geometry`: this contains the parameters for coordinate of each leg and body orientation(roll,pitch,yaw)
              - `geometry_msgs/Point32 fr` : x,y,z end effector coordinates of FR leg
              - `geometry_msgs/Point32 fl` : x,y,z end effector coordinates of FL leg
              - `geometry_msgs/Point32 br` : x,y,z end effector coordinates of BR leg
              - `geometry_msgs/Point32 bl` : x,y,z end effector coordinates of BL leg
              - `geometry_msgs/Quaternion euler_ang` : roll, pitch, yaw angles
              
  2. **`Qdog_teleop`** : this pkg creates `/hyperdog_teleop_gamepad_node. 
        - Node 1 : `/joy_node`
            This node creates commands to robot from Gamepad commands
            - subscriber : `/joy_node` 
            - publisher : `/Qdog_joy_ctrl_cmd` using the interface `Qdog_msgs/msg/JoyCtrlCmd`

  3. **`Qdog_ctrl`** : This pkg has `Body_motion_planner` and `gait_generater` and creates the nodes `/command_manager_node` and `/IK_node`
        - `Body_motion_planner` : plans body motions from control commands receive from `/command_manager_node`
        - `gait_generator` : generates gaits acording to the given gait_type command from the Gamepad 
   
        - Node 1 : `/command_manager_node` 
            - subscriber : `/Qdog_joy_ctrl_cmd` via `Qdog_msgs/msg/JoyCtrlCmds` interface
            - publisher : `/Qdog_geometry` via `Qdog_msgs/msg/Geometry` interface

        - Node 2 : `/IK_node`
            - subscriber : `/Qdog_geometry` via `Qdog_msgs/msg/Geometry` interface
            - publisher  : `/Qdog_jointController/commands`
  
  4. **`uros`** : this is the micro_ros package from its official git. this package is used to launch `micro_ros_agent` to communicate with micro-controllers which run micro_ros via ROS2
  
  5. **`Qdog_launch`** : This contains the launch file for all the above nodes and `micro_ros_agent`
  
  6. **`Qdog_gazebo_sim`** : Gazebo simmulation 
  
  7. **`Qdog_gazebo_joint_cmd`** : this pkg contains the node `/Qdog_gazebo_joint_cmd` to send joint angles to gazebo
        - Node : `/Qdog_gazebo_joint_cmd`
            - subscriber : `/Qdog_jointController/commands`
            - publisher : '/gazebo_joint_controller/commands`



