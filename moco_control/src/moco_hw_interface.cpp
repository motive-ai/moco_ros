/** \file moco_hw_interface.cpp
 * \brief Provides implementation of ROS hardware interface for Motive Moco controller
 *
 * (c) 2020 Motive Mechatronics, Inc.
 */

#include <moco_control/moco_hw_interface.h>
#include <moco_usb_manager.h>
#include <memory>

#include "moco_control/moco_actuator_state.h"
#include "moco_control/moco_actuator_system.h"
#include "moco_control/motive_robot_state.h"
#include "moco_control/motive_robot_system.h"

using namespace Motive;

namespace moco_control {

void mocoLoggingFunction(std::string msg, uint8_t error_level) {
    // trim trailing newline because ROS adds one automatically
    if (!msg.empty() && msg[msg.length() - 1] == '\n') {
        msg.erase(msg.length() - 1);
    }
    if (error_level <= 1) {
        ROS_DEBUG_STREAM_NAMED("moco_hw_interface", msg);
    } else if (error_level == 2) {
        ROS_INFO_STREAM_NAMED("moco_hw_interface", msg);
    } else if (error_level == 3) {
        ROS_WARN_STREAM_NAMED("moco_hw_interface", msg);
    } else if (error_level >= 4) {
        ROS_ERROR_STREAM_NAMED("moco_hw_interface", msg);
    }
}

MocoHWInterface::MocoHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
        : name_("moco_hw_interface"), nh_(nh), use_rosparam_joint_limits_(false),
          use_soft_limits_if_available_(false), has_error_state_(false) {
    // Check if the URDF model needs to be loaded
    if (urdf_model == NULL) {
        loadURDF(nh, "robot_description");
    } else {
        urdf_model_ = urdf_model;
    }
    // Load rosparams
    ros::NodeHandle rpnh(nh_, "hardware_interface");
    std::size_t error = 0;
    std::string actuator_state_data_topic = "moco/actuator/state";
    std::string actuator_system_data_topic = "moco/actuator/system";
    actuator_system_rate_ = 0;
    rpnh.getParam("joints", joint_names_);
    rpnh.getParam("chain", chain_name_);
    rpnh.getParam("actuator_state_data_topic", actuator_state_data_topic);
    // actuator_state rate is tied to update rate
    //rpnh.getParam("actuator_state_data_rate", );
    rpnh.getParam("actuator_system_data_topic", actuator_system_data_topic);
    rpnh.getParam("actuator_system_data_rate", actuator_system_rate_);
    rpnh.getParam("thermal_error_temperature", thermal_error_temperature_);

    std::size_t state_rate = 100;
    actuator_state_pub_ = nh_.advertise<moco_control::motive_robot_state>(actuator_state_data_topic,
            state_rate);
    if (actuator_system_rate_ > 0) {
        actuator_system_pub_ = nh.advertise<moco_control::motive_robot_system>(actuator_system_data_topic,
                                                                        actuator_system_rate_);
    }
    ROS_INFO_STREAM_NAMED(name_, "Created Moco HWInterface for chain \"" << chain_name_ << "\"");
}

bool MocoHWInterface::init() {
    // redirect Moco logging to ROS logging
    std::vector<Motive::log_sink> sinks;
    auto callback_sink = Motive::create_logging_callback_sink(&mocoLoggingFunction);
    Motive::set_logging_sink_level(callback_sink, 1);
    sinks.emplace_back(callback_sink);
    Motive::create_logger(sinks);

    bool return_value = true;
    moco_chain_ = make_unique<Chain>(chain_name_, joint_names_);
    auto state = moco_chain_->get_state();
    auto start_positions = state.joint_position;

    num_joints_ = moco_chain_->size();
    if (num_joints_ != joint_names_.size()) {
        ROS_WARN_STREAM_NAMED(name_, "Not all Moco controllers found.");
    }
    ROS_INFO_NAMED(name_, "Using %u joints", (unsigned)num_joints_);
    // Status
    joint_position_.resize(num_joints_, 0.0);
    joint_velocity_.resize(num_joints_, 0.0);
    joint_effort_.resize(num_joints_, 0.0);

    // Command
    joint_position_command_.resize(num_joints_, 0.0);
    joint_velocity_command_.resize(num_joints_, 0.0);
    joint_effort_command_.resize(num_joints_, 0.0);

    // Limits
    joint_position_lower_limits_.resize(num_joints_, 0.0);
    joint_position_upper_limits_.resize(num_joints_, 0.0);
    joint_velocity_limits_.resize(num_joints_, 0.0);
    joint_effort_limits_.resize(num_joints_, 0.0);

    // Create vector of Position-mode commands to send
    for (auto pos : start_positions) {
        auto a = std::make_shared<ActuatorPositionCommand>();
        // set to current robot position
        a->set_position_command(pos, 0, 0);
        moco_commands_.emplace_back(a);
    }

    for (int i = 0; i < moco_chain_->size(); ++i) {
        auto moco = moco_chain_->get_moco_by_index(i);
        moco_index_map_[moco->get_serial()] = i;
        moco_data_buffer_array_.emplace_back(std::make_shared<Motive::MocoShallowDataBuffer>());
    }
        // request system packets if needed
    if (actuator_system_rate_ > 0) {
        auto f = [this](std::shared_ptr<const MocoData> d) {
            this->moco_data_buffer_array_[this->moco_index_map_[d->serial()]]->write(d);
            return true;
        };
        for (int i = 0; i < moco_chain_->size(); ++i) {
            auto moco = moco_chain_->get_moco_by_index(i);
            moco->send("request", "periodic", true,
                    {{"packet_type", DATA_FMT_SYSTEM}, {"rate", actuator_system_rate_ * 2}}).get();
            moco->add_input_handler(f, DATA_FMT_SYSTEM);
        }
        ros::Duration desired_update_freq = ros::Duration(1.0 / actuator_system_rate_);
        msg_update_loop_ = nh_.createTimer(desired_update_freq, &MocoHWInterface::msgUpdate, this);
    }

    // Read values into joint_{position,velocity,effort}_
    read(ros::Time::now(), ros::Duration(0));

    // Initialize interfaces for each joint
    for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id) {
        ROS_INFO_STREAM_NAMED(name_, "Loading joint name: " << joint_names_[joint_id]);
        // Create joint state interface
        joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
            joint_names_[joint_id], &joint_position_[joint_id],
            &joint_velocity_[joint_id], &joint_effort_[joint_id]));

        // Add command interfaces to joints
        // TODO: decide based on transmissions?
        hardware_interface::JointHandle joint_handle_position = hardware_interface::JointHandle(
            joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_position_command_[joint_id]);
        position_joint_interface_.registerHandle(joint_handle_position);

        hardware_interface::JointHandle joint_handle_velocity = hardware_interface::JointHandle(
            joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_velocity_command_[joint_id]);
        velocity_joint_interface_.registerHandle(joint_handle_velocity);

        hardware_interface::PosVelJointHandle joint_handle_pos_vel = hardware_interface::PosVelJointHandle(
            joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_position_command_[joint_id],
            &joint_velocity_command_[joint_id]);
        pos_vel_joint_interface_.registerHandle(joint_handle_pos_vel);

        hardware_interface::JointHandle joint_handle_effort = hardware_interface::JointHandle(
            joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_effort_command_[joint_id]);
        effort_joint_interface_.registerHandle(joint_handle_effort);

        // Load the joint limits
        registerJointLimits(joint_handle_position, joint_handle_velocity, joint_handle_effort, joint_id);
    }  // end for each joint

    registerInterface(&joint_state_interface_);     // From RobotHW base class.
    registerInterface(&position_joint_interface_);  // From RobotHW base class.
    registerInterface(&velocity_joint_interface_);  // From RobotHW base class.
    registerInterface(&effort_joint_interface_);    // From RobotHW base class.
    registerInterface(&pos_vel_joint_interface_);    // From RobotHW base class.

    //TODO: Check if actuator needs phase lock
    for (int i = 0; i < moco_chain_->size(); ++i) {
        auto moco = moco_chain_->get_moco_by_index(i);
        auto phase_lock_mode_param_packet = moco->packet(DATA_FMT_PHASE_LOCK_MODE);
        moco->send(phase_lock_mode_param_packet);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    try {
        moco_chain_->send_command(moco_commands_);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        stopMotion();
    } catch (...) {
        ROS_ERROR_STREAM_NAMED(name_, "Could not initialize chain " << chain_name_);
        return_value = false;
    }

    if (return_value == true) {
        ROS_INFO_STREAM_NAMED(name_, "MoCo chain " << chain_name_ << " ready.");
    }
    return return_value;
}

void MocoHWInterface::read(const ros::Time& time, const ros::Duration& period) {
    moco_control::motive_robot_state state_array;
    moco_control::moco_actuator_state actuator_state;
    // get robot state
    auto state = moco_chain_->get_state();
    bool has_error = false;
    // pass back data
    for (std::size_t joint_id = 0; joint_id < moco_chain_->size(); ++joint_id) {
        joint_position_[joint_id] = state.joint_position[joint_id];
        joint_velocity_[joint_id] = state.joint_velocity[joint_id];
        joint_effort_[joint_id] = state.motor_torque[joint_id];

        // faults are bit-packed with 16 high bits as warnings, 16 low as errors
        actuator_state.error_flags = state.fault_flags[joint_id] & 0xFF;
        actuator_state.warning_flags = state.fault_flags[joint_id] >> 16;
        actuator_state.mode = state.mode[joint_id];
        actuator_state.motor_position = state.motor_position[joint_id];
        actuator_state.joint_position = state.joint_position[joint_id];
        actuator_state.motor_velocity = state.motor_velocity[joint_id];
        actuator_state.joint_velocity = state.joint_velocity[joint_id];
        actuator_state.motor_torque = state.motor_torque[joint_id];
        actuator_state.joint_torque = state.joint_torque[joint_id];
        actuator_state.joint_torque_dot = state.joint_torque_dot[joint_id];
        state_array.robot_state.push_back(actuator_state);
        if (actuator_state.error_flags != 0) {
            ROS_ERROR_STREAM_THROTTLE_NAMED(1, name_, "Error in joint_id " << joint_id
                << ": " << actuator_state.error_flags);
            has_error = true;
        }
    }
    has_error_state_ = has_error;
    actuator_state_pub_.publish(state_array);
}

void MocoHWInterface::msgUpdate(const ros::TimerEvent& e) {
    if (actuator_system_rate_ > 0) {
      publishSystemState();
    }
}

void MocoHWInterface::publishSystemState() {
    moco_control::motive_robot_system system_array;
    moco_control::moco_actuator_system actuator_system;
    float t;
    for (std::size_t joint_id = 0; joint_id < moco_chain_->size(); ++joint_id) {
        auto system_data = moco_data_buffer_array_[joint_id]->pop(DATA_FMT_SYSTEM);
        if (system_data == nullptr) {
            return;
        }
        auto status_data = system_data->generic_data().status.system;
        actuator_system.bus_voltage = status_data.bus_voltage;
        // TODO: get calibrated data instead of raw
        t = status_data.cpu_temp;
        actuator_system.cpu_temp = 23.3 + 7.53*t;
        t = status_data.external_temp;
        actuator_system.external_temp = -2777 + 2.71*t + -8.71e-4*t*t + 9.34e-08*t*t*t;
        system_array.robot_system.push_back(actuator_system);

        if (actuator_system.external_temp > thermal_error_temperature_) {
            auto board_name = moco_chain_->get_moco_by_index(joint_id)->get_board_name();
            ROS_ERROR_STREAM_NAMED(name_, "Thermal error on joint " <<
                board_name.chain << ":" << board_name.name << std::setprecision(5) <<
                actuator_system.external_temp << " degrees C");
            setEstop(joint_id);
            stopMotion();
        } else if (actuator_system.external_temp > thermal_error_temperature_ - 5) {
            auto board_name = moco_chain_->get_moco_by_index(joint_id)->get_board_name();
            ROS_WARN_STREAM_THROTTLE_NAMED(10, name_, "Near thermal limit on joint " <<
                board_name.chain << ":" << board_name.name << " - " << std::setprecision(5) <<
                actuator_system.external_temp << " degrees C (limit of " <<
                thermal_error_temperature_ << " C)");
        }
    }
    actuator_system_pub_.publish(system_array);
}

void MocoHWInterface::write(const ros::Time& time, const ros::Duration& period) {
    enforceLimits(period);

    if (has_error_state_) {
        ROS_ERROR_STREAM_THROTTLE_NAMED(1, name_, "Not sending actuator commands because of joint error state");
        return;
    }

    // update command to send
    for (std::size_t joint_id = 0; joint_id < moco_chain_->size(); ++joint_id) {
        auto cmd = dynamic_cast<ActuatorPositionCommand*>(moco_commands_[joint_id].get());
        // Second-to-last argument is the maximum velocity that will be used to seek the target pose
        cmd->set_position_command(static_cast<float>(joint_position_command_[joint_id]), 1, 0);
    }

    try {
        // send update to all actuators
        moco_chain_->send_command(moco_commands_);
    } catch (std::runtime_error &e) {
        ROS_ERROR_STREAM_THROTTLE_NAMED(1, name_, "Could not set joint position: " << e.what());
    }
}

void MocoHWInterface::registerJointLimits(const hardware_interface::JointHandle &joint_handle_position,
                                             const hardware_interface::JointHandle &joint_handle_velocity,
                                             const hardware_interface::JointHandle &joint_handle_effort,
                                             std::size_t joint_id) {
    // Default values
    joint_position_lower_limits_[joint_id] = -std::numeric_limits<double>::max();
    joint_position_upper_limits_[joint_id] = std::numeric_limits<double>::max();
    joint_velocity_limits_[joint_id] = std::numeric_limits<double>::max();
    joint_effort_limits_[joint_id] = std::numeric_limits<double>::max();

    // Limits datastructures
    joint_limits_interface::JointLimits joint_limits;     // Position
    joint_limits_interface::SoftJointLimits soft_limits;  // Soft Position
    bool has_joint_limits = false;
    bool has_soft_limits = false;

    // Get limits from URDF
    if (urdf_model_ == NULL) {
        ROS_WARN_STREAM_NAMED(name_, "No URDF model loaded, unable to get joint limits");
        return;
    }

    // Get limits from URDF
    urdf::JointConstSharedPtr urdf_joint = urdf_model_->getJoint(joint_names_[joint_id]);

    // Get main joint limits
    if (urdf_joint == NULL) {
        ROS_ERROR_STREAM_NAMED(name_, "URDF joint not found " << joint_names_[joint_id]);
        return;
    }

    // Get limits from URDF
    if (joint_limits_interface::getJointLimits(urdf_joint, joint_limits)) {
        has_joint_limits = true;
        ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " has URDF position limits ["
                                                                << joint_limits.min_position << ", "
                                                                << joint_limits.max_position << "]");
        if (joint_limits.has_velocity_limits) {
          ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " has URDF velocity limit ["
                                                                  << joint_limits.max_velocity << "]");
        }
    } else {
    if (urdf_joint->type != urdf::Joint::CONTINUOUS)
      ROS_WARN_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " does not have a URDF "
                            "position limit");
    }

    // Get limits from ROS param
    if (use_rosparam_joint_limits_) {
        if (joint_limits_interface::getJointLimits(joint_names_[joint_id], nh_, joint_limits)) {
            has_joint_limits = true;
            ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id]
                                          << " has rosparam position limits ["
                                          << joint_limits.min_position << ", "
                                          << joint_limits.max_position << "]");
            if (joint_limits.has_velocity_limits) {
                ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id]
                                                << " has rosparam velocity limit ["
                                                << joint_limits.max_velocity << "]");
            }
        }  // the else debug message provided internally by joint_limits_interface
    }

    // Get soft limits from URDF
    if (use_soft_limits_if_available_) {
        if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits)) {
          has_soft_limits = true;
          ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " has soft joint limits.");
        } else {
          ROS_DEBUG_STREAM_NAMED(name_, "Joint " << joint_names_[joint_id] << " does not have soft joint limits");
        }
    }

    // Quit we we haven't found any limits in URDF or rosparam server
    if (!has_joint_limits) {
        return;
    }

    // Copy position limits if available
    if (joint_limits.has_position_limits) {
        // Slighly reduce the joint limits to prevent floating point errors
        joint_limits.min_position += std::numeric_limits<double>::epsilon();
        joint_limits.max_position -= std::numeric_limits<double>::epsilon();

        joint_position_lower_limits_[joint_id] = joint_limits.min_position;
        joint_position_upper_limits_[joint_id] = joint_limits.max_position;
    }

    // Copy velocity limits if available
    if (joint_limits.has_velocity_limits) {
        joint_velocity_limits_[joint_id] = joint_limits.max_velocity;
    }

    // Copy effort limits if available
    if (joint_limits.has_effort_limits) {
        joint_effort_limits_[joint_id] = joint_limits.max_effort;
    }

    if (has_soft_limits) {  // Use soft limits
        ROS_DEBUG_STREAM_NAMED(name_, "Using soft saturation limits");
        const joint_limits_interface::PositionJointSoftLimitsHandle soft_handle_position(joint_handle_position, joint_limits, soft_limits);
        pos_jnt_soft_limits_.registerHandle(soft_handle_position);
        const joint_limits_interface::VelocityJointSoftLimitsHandle soft_handle_velocity(joint_handle_velocity, joint_limits, soft_limits);
        vel_jnt_soft_limits_.registerHandle(soft_handle_velocity);
        const joint_limits_interface::EffortJointSoftLimitsHandle soft_handle_effort(joint_handle_effort, joint_limits, soft_limits);
        eff_jnt_soft_limits_.registerHandle(soft_handle_effort);
    } else {  // Use saturation limits

        ROS_DEBUG_STREAM_NAMED(name_, "Using saturation limits (not soft limits)");

        const joint_limits_interface::PositionJointSaturationHandle sat_handle_position(joint_handle_position, joint_limits);
        pos_jnt_sat_interface_.registerHandle(sat_handle_position);

        const joint_limits_interface::VelocityJointSaturationHandle sat_handle_velocity(joint_handle_velocity, joint_limits);
        vel_jnt_sat_interface_.registerHandle(sat_handle_velocity);

        const joint_limits_interface::EffortJointSaturationHandle sat_handle_effort(joint_handle_effort, joint_limits);
        eff_jnt_sat_interface_.registerHandle(sat_handle_effort);
    }
}

void MocoHWInterface::reset() {
    // Reset joint limits state, in case of mode switch or e-stop
    pos_jnt_sat_interface_.reset();
    pos_jnt_soft_limits_.reset();
}

void MocoHWInterface::enforceLimits(const ros::Duration &period) {
    // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
    // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
    // DEPENDING ON YOUR CONTROL METHOD
    //
    // EXAMPLES:
    //
    // Saturation Limits ---------------------------
    //
    // Enforces position and velocity
    pos_jnt_sat_interface_.enforceLimits(period);
    //
    // Enforces velocity and acceleration limits
    // vel_jnt_sat_interface_.enforceLimits(period);
    //
    // Enforces position, velocity, and effort
    // eff_jnt_sat_interface_.enforceLimits(period);

    // Soft limits ---------------------------------
    //
    // pos_jnt_soft_limits_.enforceLimits(period);
    // vel_jnt_soft_limits_.enforceLimits(period);
    // eff_jnt_soft_limits_.enforceLimits(period);
    //
    // ----------------------------------------------------
    // ----------------------------------------------------
    // ----------------------------------------------------
}

void MocoHWInterface::loadURDF(ros::NodeHandle &nh, std::string param_name) {
    std::string urdf_string;
    urdf_model_ = new urdf::Model();

    // search and wait for robot_description on param server
    while (urdf_string.empty() && ros::ok()) {
        std::string search_param_name;
        if (nh.searchParam(param_name, search_param_name)) {
          ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
                                nh.getNamespace() << search_param_name);
          nh.getParam(search_param_name, urdf_string);
        } else {
          ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
                                nh.getNamespace() << param_name);
          nh.getParam(param_name, urdf_string);
        }

        usleep(100000);
    }

    if (!urdf_model_->initString(urdf_string)) {
        ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
    } else {
        ROS_DEBUG_STREAM_NAMED(name_, "Received URDF from param server");
    }
}

void MocoHWInterface::stopMotion() {
    ROS_WARN_STREAM_NAMED(name_, "Stopping motion of all joints");
    // get robot state
    auto state = moco_chain_->get_state();

    // set joint command to current state
    for (std::size_t joint_id = 0; joint_id < moco_chain_->size(); ++joint_id) {
        joint_position_command_[joint_id] = state.joint_position[joint_id];
        joint_velocity_command_[joint_id] = state.joint_velocity[joint_id];
        joint_effort_command_[joint_id] = state.motor_torque[joint_id];
    }

    // send updated joint command to actuators
    write(ros::Time::now(), ros::Duration(0));
}

void MocoHWInterface::setEstop(int joint_id) {

    auto moco = moco_chain_->get_moco_by_index(joint_id);
    if (moco == nullptr) {
        ROS_ERROR_STREAM_NAMED(name_, "Invalid joint_id: " << joint_id);
    }
    moco->send("request", "system", true, {{"estop_control", 1}, {"estop_engage", 1}}).get();
    auto board_name = moco->get_board_name();
    ROS_WARN_STREAM_NAMED(name_, "E-Stopping joint " << board_name.chain << ":" << board_name.name);
}


}  // namespace
