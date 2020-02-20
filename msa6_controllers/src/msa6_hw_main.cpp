
#include <msa6_controllers/msa6_hw_control_loop.h>
#include <msa6_controllers/msa6_hw_interface.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "msa6_hw_interface");
    ros::NodeHandle nh;

    // NOTE: We run the ROS loop in a separate thread as external calls such
    // as service callbacks to load controllers can block the (main) control loop
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Create the hardware interface specific to your robot
    std::shared_ptr<msa6_controllers::MocoHWInterface> msa6_hw_interface(new msa6_controllers::MocoHWInterface(nh));
    msa6_hw_interface->init();

    // Start the control loop
    msa6_controllers::MocoHWControlLoop control_loop(nh, msa6_hw_interface);

    // Wait until shutdown signal recieved
    ros::waitForShutdown();

    return 0;
}
