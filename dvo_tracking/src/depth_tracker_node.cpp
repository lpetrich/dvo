/**
 *  This file is part of dvo.
 *
 *  Copyright 2012 Christian Kerl <christian.kerl@in.tum.de> (Technical University of Munich)
 *  For more information see <http://vision.in.tum.de/data/software/dvo>.
 *
 *  dvo is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  dvo is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with dvo.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <dvo_tracking/camera_dense_tracking.h>

#include <dvo/core/macros.h>
#line __LINE__ "depth_tracker_node.cpp"

int main(int argc, char **argv) {
    TRACE()
    ros::init(argc, argv, "depth_tracker");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    std::cout << "\n\tNode initialized, initializing dense tracker...\n";
    dvo_tracking::CameraDenseTracker dense_tracker(nh, nh_private);
    std::cout << "\n\tDense tracker initialized, starting multi-threaded spinner\n";

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}
