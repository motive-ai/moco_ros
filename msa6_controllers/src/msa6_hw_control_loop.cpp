#include <msa6_controllers/msa6_hw_control_loop.h>

namespace {
    const std::string LOGNAME = "msa6_hw_control_loop";
    // Parameters are loaded in this namespace
    const std::string PARAM_NAMESPACE = "msa6_hw_control_loop";
}

namespace msa6_controllers {

MocoHWControlLoop::MocoHWControlLoop(ros::NodeHandle& nh,
    std::shared_ptr<msa6_controllers::MocoHWInterface> hardware_interface)
    : nh_(nh), hardware_interface_(hardware_interface) {
    // Create the controller manager
    controller_manager_.reset(new controller_manager::ControllerManager(hardware_interface_.get(), nh_));

    // Load rosparams with rosparam_shortcuts. It kills the node if a parameter isn't found.
    std::size_t rosparam_error = 0;
    rosparam_error += !rosparam_shortcuts::get("", nh_, PARAM_NAMESPACE + "/loop_hz", loop_hz_);
    ROS_INFO_NAMED(LOGNAME, "Using loop_hz of %.5lf", loop_hz_);
    rosparam_error += !rosparam_shortcuts::get("", nh_, PARAM_NAMESPACE + "/cycle_time_error_threshold",
        cycle_time_error_threshold_);
    ROS_INFO_NAMED(LOGNAME, "Using threshold of %.5lf", cycle_time_error_threshold_);

    // Get current time for use with first update
    clock_gettime(CLOCK_MONOTONIC, &last_time_);

    // Start timer
    ros::Duration desired_update_freq = ros::Duration(1 / loop_hz_);
    non_realtime_loop_ = nh_.createTimer(desired_update_freq, &MocoHWControlLoop::update, this);
}

void MocoHWControlLoop::update(const ros::TimerEvent& e) {
    // Get change in time
    clock_gettime(CLOCK_MONOTONIC, &current_time_);
    elapsed_time_ =
      ros::Duration(current_time_.tv_sec - last_time_.tv_sec + (current_time_.tv_nsec - last_time_.tv_nsec) / 1.0e9);
    last_time_ = current_time_;
    // ROS_DEBUG_STREAM_THROTTLE_NAMED(1, "msa6_hw_main","Sampled update with elapsed time " << elapsed_time_.toSec());

    // Error check cycle time
    const double cycle_time_error = (elapsed_time_ - desired_update_freq_).toSec();
    if (cycle_time_error > cycle_time_error_threshold_) {
        ROS_WARN_STREAM_NAMED(name_, "Cycle time exceeded error threshold by: "
                                         << cycle_time_error << ", cycle time: " << elapsed_time_
                                         << ", threshold: " << cycle_time_error_threshold_);
    }

    // Input
    hardware_interface_->read(elapsed_time_);

    // Control
    controller_manager_->update(ros::Time::now(), elapsed_time_);

    // Output
    hardware_interface_->write(elapsed_time_);
}

}  // namespace
