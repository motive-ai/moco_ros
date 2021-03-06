#include <moco_control/moco_hw_control_loop.h>

namespace moco_control {

MocoHWControlLoop::MocoHWControlLoop(ros::NodeHandle& nh,
    std::shared_ptr<moco_control::MocoHWInterface> hardware_interface)
    : nh_(nh), hardware_interface_(hardware_interface) {
    // Create the controller manager
    controller_manager_.reset(new controller_manager::ControllerManager(hardware_interface_.get(), nh_));

    // Load rosparams
    ros::NodeHandle rpsnh(nh, name_);
    std::size_t error = 0;
    if (!rpsnh.getParam("loop_hz", loop_hz_)) {
        loop_hz_ = 100.0;
    }
    if (!rpsnh.getParam("cycle_time_error_threshold", cycle_time_error_threshold_)) {
        cycle_time_error_threshold_ = .1;
    }
    ROS_INFO_NAMED("moco_hw_control_loop", "Using cycle time threshold of %.5lf", cycle_time_error_threshold_);
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
    ROS_DEBUG_STREAM_THROTTLE_NAMED(1, "moco_hw_main","Sampled update with elapsed time " << elapsed_time_.toSec());

    // Error check cycle time
    const double cycle_time_error = (elapsed_time_ - desired_update_freq_).toSec();
    if (cycle_time_error > cycle_time_error_threshold_) {
        ROS_WARN_STREAM_NAMED(name_, "Cycle time exceeded error threshold by: "
                                         << cycle_time_error << ", cycle time: " << elapsed_time_
                                         << ", threshold: " << cycle_time_error_threshold_);
    }
    auto curr_time_ = ros::Time::now();
    // Input
    hardware_interface_->read(curr_time_, elapsed_time_);

    // Control
    controller_manager_->update(curr_time_, elapsed_time_);

    // Output
    hardware_interface_->write(curr_time_, elapsed_time_);
}

}  // namespace
