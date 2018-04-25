/* lpetrich 23/04/18 /**/

#include <ros/ros.h>
#include <ros/console.h>
#include <dvo/core/macros.h>

#line __LINE__ "depth_node.cpp"

int main(int argc, char **argv) {
    TRACE()
    ros::init(argc, argv, "depth_node");
    ros::NodeHandle nh;
    CameraDenseTracker dense_tracker(nh, nh_private);
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}
