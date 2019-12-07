/**
 * \author Aytac Kahveci
*/
#ifndef GAZEBO_CLOSED_LOOP_HW_SIM_H_
#define GAZEBO_CLOSED_LOOP_HW_SIM_H_

// ros_control
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

// gazebo_ros_control
#include <gazebo_ros_control/robot_hw_sim.h>

// URDF
#include <urdf/model.h>

namespace gazebo_ros_control
{

class ClosedLoopHWSim: public gazebo_ros_control::RobotHWSim
{
public:

    virtual bool initSim(const std::string& robot_namespace,
                         ros::NodeHandle model_nh,
                         gazebo::physics::ModelPtr parent_model,
                         const urdf::Model *const urdf_model,
                         std::vector<transmission_interface::TransmissionInfo> transmissions);

    virtual void readSim(ros::Time time, ros::Duration period);

    virtual void writeSim(ros::Time time, ros::Duration period);

    virtual void eStopActive(const bool active);

protected:
    enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID, PASSIVE};

    void registerJointLimits(const std::string& joint_name,
                             const hardware_interface::JointHandle& joint_handle,
                             const ControlMethod ctrl_method,
                             const ros::NodeHandle& joint_limit_nh,
                             const urdf::Model *const urdf_model,
                             int *const joint_type, double *const lower_limit,
                             double *const upper_limit, double *const effort_limit);

    unsigned int n_dof_;

    hardware_interface::JointStateInterface js_interface_;
    hardware_interface::EffortJointInterface ej_interface_;
    hardware_interface::PositionJointInterface pj_interface_;
    hardware_interface::VelocityJointInterface vj_interface_;

    joint_limits_interface::EffortJointSaturationInterface ej_sat_interface_;
    joint_limits_interface::EffortJointSoftLimitsInterface ej_limits_interface_;
    joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
    joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;
    joint_limits_interface::VelocityJointSaturationInterface vj_sat_interface_;
    joint_limits_interface::VelocityJointSoftLimitsInterface vj_limits_interface_;

    std::vector<std::string> joint_names_;
    std::vector<int> joint_types_;
    std::vector<double> joint_lower_limits_;
    std::vector<double> joint_upper_limits_;
    std::vector<double> joint_effort_limits_;
    std::vector<ControlMethod> joint_control_methods_;
    std::vector<control_toolbox::Pid> pid_controllers_;
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;
    std::vector<double> joint_effort_command_;
    std::vector<double> joint_position_command_;
    std::vector<double> last_joint_position_command_;
    std::vector<double> joint_velocity_command_;

    std::vector<gazebo::physics::JointPtr> sim_joints_;
    
    std::string physics_type_;

    bool e_stop_active_, last_e_stop_active_;
};

typedef boost::shared_ptr<ClosedLoopHWSim> ClosedLoopHWSimPtr;

} // namespace gazebo_ros_control

#endif