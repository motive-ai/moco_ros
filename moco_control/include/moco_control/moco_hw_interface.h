/** \file moco_hw_interface.h
 * \brief ROS hardware interface for Motive Moco controller
 *
 * (c) 2020 Motive Mechatronics, Inc.
 */
#ifndef MOCO_HW_INTERFACE_H
#define MOCO_HW_INTERFACE_H

// C++
#include <vector>
#include <map>
#include <boost/scoped_ptr.hpp>

// ROS
#include <ros/ros.h>
#include <urdf/model.h>

// ROS Controls
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

// Moco
#include <chain.h>

namespace moco_control {

    template<typename T, typename... Args>
    std::unique_ptr<T> make_unique(Args&&... args)
    {
        return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
    }

    /// \brief Callback function for Motive log messages to get translated to ROS
    void mocoLoggingFunction(std::string msg, uint8_t error_level);

/// \brief Hardware interface for a robot
class MocoHWInterface : public hardware_interface::RobotHW {
  public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
    MocoHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

    virtual ~MocoHWInterface()
    {
      // Ensure motion stops before exiting
      stopMotion();
    }

    /** \brief Initialize the hardware interface */
    virtual bool init();

    /** \brief Read the state from the robot hardware. */
    virtual void read(const ros::Time& time, const ros::Duration& period);

    /** \brief Write the command to the robot hardware. */
    virtual void write(const ros::Time& time, const ros::Duration& period);

    /** \brief Set all members to default values */
    virtual void reset();

    /**
     * \brief Register the limits of the joint specified by joint_id and joint_handle. The limits
     * are retrieved from the urdf_model.
     *
     * \return the joint's type, lower position limit, upper position limit, and effort limit.
    */
    virtual void registerJointLimits(const hardware_interface::JointHandle &joint_handle_position,
                           const hardware_interface::JointHandle &joint_handle_velocity,
                           const hardware_interface::JointHandle &joint_handle_effort,
                           std::size_t joint_id);

    /** \breif Enforce limits for all values before writing */
    virtual void enforceLimits(const ros::Duration &period);

    /** \brief Halt the robot */
    virtual void stopMotion();

    /** \brief Trigger soft E-stop for joint */
    void setEstop(int joint_id);

  protected:

    /** \brief Get the URDF XML from the parameter server */
    virtual void loadURDF(ros::NodeHandle& nh, std::string param_name);

    /// \brief Read status packets from the actuators and send ROS messages
    void msgUpdate(const ros::TimerEvent& e);

    /// \brief send contents of SystemStatus as ROS message
    void publishSystemState();

    // Short name of this class
    std::string name_;

    // Startup and shutdown of the internal node inside a roscpp program
    ros::NodeHandle nh_;

    // Hardware interfaces
    hardware_interface::PositionJointInterface position_joint_interface_;
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    hardware_interface::EffortJointInterface effort_joint_interface_;
    hardware_interface::PosVelJointInterface pos_vel_joint_interface_;

    // Joint limits interfaces - Saturation
    joint_limits_interface::PositionJointSaturationInterface pos_jnt_sat_interface_;
    joint_limits_interface::VelocityJointSaturationInterface vel_jnt_sat_interface_;
    joint_limits_interface::EffortJointSaturationInterface eff_jnt_sat_interface_;

    // Joint limits interfaces - Soft limits
    joint_limits_interface::PositionJointSoftLimitsInterface pos_jnt_soft_limits_;
    joint_limits_interface::VelocityJointSoftLimitsInterface vel_jnt_soft_limits_;
    joint_limits_interface::EffortJointSoftLimitsInterface eff_jnt_soft_limits_;

    // Configuration
    std::vector<std::string> joint_names_;
    float thermal_error_temperature_;
    bool has_error_state_;

    // Name of Moco chain
    std::string chain_name_;
    // Number of joints in config
    std::size_t num_joints_;
    urdf::Model *urdf_model_;

    // Modes
    bool use_rosparam_joint_limits_;
    bool use_soft_limits_if_available_;

    // States
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;

    // Commands
    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocity_command_;
    std::vector<double> joint_effort_command_;

    // Copy of limits, in case we need them later in our control stack
    std::vector<double> joint_position_lower_limits_;
    std::vector<double> joint_position_upper_limits_;
    std::vector<double> joint_velocity_limits_;
    std::vector<double> joint_effort_limits_;

    std::unique_ptr<Motive::Chain> moco_chain_;
    std::vector<std::shared_ptr<Motive::ActuatorCommand>> moco_commands_;
    std::vector<std::shared_ptr<Motive::MocoShallowDataBuffer>> moco_data_buffer_array_;
    std::map<Motive::Serial, int> moco_index_map_;

    //msgs
    ros::Publisher actuator_state_pub_;
    ros::Publisher actuator_system_pub_;
    int actuator_system_rate_;

    ros::Timer msg_update_loop_;

};  // class

}  // namespace

#endif
