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

#ifndef OBJECT_TRAJECTORY_VISUALIZER_H_
#define OBJECT_TRAJECTORY_VISUALIZER_H_


#include <Eigen/Geometry>
#include <boost/function.hpp>

#include <dvo/core/rgbd_image.h>
#include <dvo/util/fluent_interface.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <dvo/visualization/async_point_cloud_builder.h>
#include <dvo/visualization/point_cloud_aggregator.h>
#include <pcl_ros/point_cloud.h>

#include <interactive_markers/interactive_marker_server.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>

#include <eigen_conversions/eigen_msg.h>
#include <valgrind/memcheck.h>
#include <dvo/visualization/camera_trajectory_visualizer.h>

#include <ros/ros.h>
#include <dvo/core/macros.h>

namespace dvo
{
namespace visualization
{
namespace internal
{

struct ObjectTrajectoryVisualizerImpl;
} /* namespace internal */

class ObjectTrajectoryVisualizer : public dvo::visualization::CameraTrajectoryVisualizerInterface
{
public:
	ObjectTrajectoryVisualizer(ros::NodeHandle& nh);
	virtual ~ObjectTrajectoryVisualizer();

	virtual dvo::visualization::CameraVisualizer::Ptr camera(std::string name);
	virtual dvo::visualization::TrajectoryVisualizer::Ptr trajectory(std::string name);

	virtual void reset();

	virtual bool native(void*& native_visualizer);
private:
	internal::ObjectTrajectoryVisualizerImpl* impl_;
};

} /* namespace visualization */
} /* namespace dvo */
#endif /* OBJECT_TRAJECTORY_VISUALIZER_H_ */
