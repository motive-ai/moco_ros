#include <msa6_controllers/msa6_hw_interface.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <time.h>

namespace msa6_controllers {

/**
 * \brief The control loop - repeatidly calls read() and write() to the hardware interface at a
 * specified frequency
 *        We use MONOTONIC time to ensure robustness in the event of system time updates/change.
 *        See
 * http://stackoverflow.com/questions/3523442/difference-between-clock-realtime-and-clock-monotonic
 */
class MocoHWControlLoop {
  public:
    /**
    * \brief Constructor
    * \param NodeHandle
    * \param hardware_interface - the robot-specific hardware interface to be use with your robot
    */
    MocoHWControlLoop(
      ros::NodeHandle& nh,
      std::shared_ptr<msa6_controllers::MocoHWInterface> hardware_interface);

    /** \brief Timer event
    *         Note: we do not use the TimerEvent time difference because it does NOT guarantee that
    * the time source is
    *         strictly linearly increasing
    */
    void update(const ros::TimerEvent& e);

  protected:
    // Startup and shutdown of the internal node inside a roscpp program
    ros::NodeHandle nh_;

    // Name of this class
    std::string name_ = "msa6_hw_control_loop";

    // Settings
    ros::Duration desired_update_freq_;
    double cycle_time_error_threshold_;

    // Timing
    ros::Timer non_realtime_loop_;
    ros::Duration elapsed_time_;
    double loop_hz_;
    struct timespec last_time_;
    struct timespec current_time_;

    /** \brief ROS Controller Manager and Runner
     *
     * This class advertises a ROS interface for loading, unloading, starting, and
     * stopping ros_control-based controllers. It also serializes execution of all
     * running controllers in \ref update.
    */
    std::shared_ptr<controller_manager::ControllerManager> controller_manager_;

    /** \brief Abstract Hardware Interface for your robot */
    std::shared_ptr<msa6_controllers::MocoHWInterface> hardware_interface_;

};  // end class

}  // namespace
