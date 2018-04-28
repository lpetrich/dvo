/* lpetrich /**/
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

#ifndef CAMERA_DENSE_TRACKING_H_
#define CAMERA_DENSE_TRACKING_H_

#include <ros/ros.h>
#include <ros/console.h>
// #include <dynamic_reconfigure/server.h>
#include <std_msgs/UInt32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include <opencv2/opencv.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unistd.h>
#include <iostream>

#include <dvo/dense_tracking.h>
#include <dvo/util/stopwatch.h>
#include <dvo/visualization/camera_trajectory_visualizer.h>

#include <dvo/core/datatypes.h>
#include <dvo/core/intrinsic_matrix.h>
#include <dvo/core/rgbd_image.h>
#include <dvo/core/least_squares.h>
#include <dvo/core/weight_calculation.h>
#include <dvo/core/macros.h>
#include <dvo/core/surface_pyramid.h>

#include <dvo_tracking/camera_base.h>
#include <dvo_tracking/CameraDenseTrackerConfig.h>
#include <dvo_tracking/util/util.h>
// #include <dvo_tracking/util/configtools.h>
// #include <dvo_tracking/visualization/ros_camera_trajectory_visualizer.h>

namespace dvo_tracking
{

class ObjectTracker : public CameraBase
{
private:
	// typedef dynamic_reconfigure::Server<dvo_tracking::CameraDenseTrackerConfig> ReconfigureServer;
	uint32_t width;
	uint32_t height;
	double u;
	double v;
	cv::Mat current_view;
	boost::shared_ptr<dvo::DenseTracker> tracker;
	dvo::DenseTracker::Config tracker_cfg;
	// dvo::core::RgbdCameraPyramidPtr camera;
	// dvo::core::RgbdImagePyramidPtr current, reference;
  	boost::shared_ptr<dvo::core::RgbdImagePyramid> current, reference;

	Eigen::Affine3d accumulated_transform, from_baselink_to_kinect, latest_absolute_transform_;

	size_t frames_since_last_success;

	tf::TransformListener tl;
	ros::Publisher uv_pub_;
	ros::Subscriber uv_sub_;

	dvo::visualization::CameraTrajectoryVisualizerInterface* vis_;
	bool use_dense_tracking_estimate_;
	boost::mutex tracker_mutex_;

	bool hasChanged(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg);
	void reset(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg);
	void uvPublish(const std_msgs::Header& header, const Eigen::Affine3d& transform, const std::string frame);

public:
	ObjectTracker(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
	virtual ~ObjectTracker();

	virtual void handleImages(
			const sensor_msgs::Image::ConstPtr& rgb_image_msg,
			const sensor_msgs::Image::ConstPtr& depth_image_msg,
			const sensor_msgs::CameraInfo::ConstPtr& rgb_camera_info_msg,
			const sensor_msgs::CameraInfo::ConstPtr& depth_camera_info_msg
	);
	void uvCallback(const geometry_msgs::Point::ConstPtr& data);
	void handleConfig(dvo_tracking::CameraDenseTrackerConfig& config, uint32_t level);
};

} /* namespace dvo_tracking */
#endif /* CAMERA_DENSE_TRACKING_H_ */
