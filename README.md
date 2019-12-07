# gazebo_closed_loop_hw_sim
Enables taking states of the passive joints from Gazebo.

This package is a hardware interface for gazebo_ros_control plugin which enables taking joint states of passive joints. It can be used to simulate closed chain structures in URDF because of URDF currently does not support closed chains. 
Closed chain structure should be divided into the sets of serial chains in URDF model and close chain structure should be built inside gazebo tags. See four_bar.urdf
