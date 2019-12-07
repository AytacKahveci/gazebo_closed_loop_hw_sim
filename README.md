# gazebo_closed_loop_hw_sim
Enables taking states of the passive joints from Gazebo.

This package is a hardware interface for gazebo_ros_control plugin which enables taking joint states of passive joints. It can be used to simulate closed chain structures in URDF because of URDF currently does not support closed chains. 
Closed chain structure should be divided into the sets of serial chains in URDF model and close chain structure should be built inside gazebo tags. See four_bar.urdf

[![gazebo_closed_loop_hw_sim](https://img.youtube.com/vi/5O9mDumzZUw/0.jpg)](https://www.youtube.com/watch?v=5O9mDumzZUw)

<H1> Example Simulation </H1>

In order to start example simulation:

  <code> roslaunch gazebo_closed_loop_hw_sim gazebo.launch </code>

Then start simulation from Gazebo and launch position controller:

  <code> roslaunch gazebo_closed_loop_hw_sim controller.launch </code>
  
Set point can be given to the joint position controller with ros publisher:
  
  <code> rostopic pub -1 /joint_position_controller/command std_msgs/Float64 "data: 0.1" </code>
