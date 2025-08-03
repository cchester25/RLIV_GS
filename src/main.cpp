#include "rliv_gs/rlivgs_system.hpp"

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "mapping");
    ros::NodeHandle nh;

    // Initialize system
    rlivgs::RlivgsSystem rlivgs_system(nh);
    
    // Run the system
    rlivgs_system.run();

    return 0;
}