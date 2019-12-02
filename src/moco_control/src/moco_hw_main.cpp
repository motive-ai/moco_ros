
#include <moco_control/moco_hw_control_loop.h>
#include <moco_control/moco_hw_interface.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "moco_hw_interface");
    ros::NodeHandle nh;

    // NOTE: We run the ROS loop in a separate thread as external calls such
    // as service callbacks to load controllers can block the (main) control loop
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Create the hardware interface specific to your robot
    std::shared_ptr<moco_control::MocoHWInterface> moco_hw_interface(new moco_control::MocoHWInterface(nh));
    moco_hw_interface->init();

    // Start the control loop
    moco_control::MocoHWControlLoop control_loop(nh, moco_hw_interface);

    // Wait until shutdown signal recieved
    ros::waitForShutdown();

    return 0;
}
