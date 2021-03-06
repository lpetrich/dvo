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

#include <boost/bind.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <dvo/core/surface_pyramid.h>
#include <dvo/util/stopwatch.h>

#include <dvo_tracking/camera_dense_tracking.h>
#include <dvo_tracking/util/util.h>
#include <dvo_tracking/util/configtools.h>
#include <dvo_tracking/visualization/ros_camera_trajectory_visualizer.h>

#include <unistd.h>
// #include <valgrind/memcheck.h>

#include <iostream>
#line __LINE__ "camera_dense_tracking.cpp"

namespace dvo_tracking
{

using namespace dvo;
using namespace dvo::core;
using namespace dvo::util;

CameraDenseTracker::CameraDenseTracker(ros::NodeHandle& nh, ros::NodeHandle& nh_private) :
	CameraBase(nh, nh_private),
	tracker_cfg(DenseTracker::getDefaultConfig()),
	frames_since_last_success(0),
	reconfigure_server_(nh_private),
	vis_(new dvo::visualization::NoopCameraTrajectoryVisualizer()),
	use_dense_tracking_estimate_(false)
{
  	TRACE()
	width = 0;
	height = 0;
	frames_since_last_success = 0;

	pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("rgbd/pose", 1);

	ReconfigureServer::CallbackType reconfigure_server_callback = boost::bind(&CameraDenseTracker::handleConfig, this, _1, _2);
	reconfigure_server_.setCallback(reconfigure_server_callback);

	dvo_tracking::util::tryGetTransform(from_baselink_to_kinect, tl, "/camera_link", "/camera_rgb_optical_frame");

	// std::cout << "\nTRANSFORM: camera_link -> camera_rgb_optical_frame\n" << from_baselink_to_kinect.matrix() << "\n\n";

	pose_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("pelican/pose", 1, &CameraDenseTracker::handlePose, this);

	latest_absolute_transform_.setIdentity();
	accumulated_transform.setIdentity();
///// ORIGINALLY COMMENTED OUT
	// dvo::visualization::Visualizer::instance()
	//  .enabled(false)
	//  .useExternalWaitKey(false)
	//  .save(false)
	// ;
}

CameraDenseTracker::~CameraDenseTracker()
{
  	TRACE()
	delete vis_;
}

bool CameraDenseTracker::hasChanged(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg)
{
  	TRACE()

	return width != camera_info_msg->width || height != camera_info_msg->height;
}

void CameraDenseTracker::reset(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg)
{
  	TRACE()
	//IntrinsicMatrix intrinsics = IntrinsicMatrix::create(camera_info_msg->K[0], camera_info_msg->K[4], camera_info_msg->K[2], camera_info_msg->K[5]);
	IntrinsicMatrix intrinsics = IntrinsicMatrix::create(camera_info_msg->P[0], camera_info_msg->P[5], camera_info_msg->P[2], camera_info_msg->P[6]);

	camera.reset(new dvo::core::RgbdCameraPyramid(camera_info_msg->width, camera_info_msg->height, intrinsics));
	camera->build(tracker_cfg.getNumLevels());

	tracker.reset(new DenseTracker(tracker_cfg));

	static RgbdImagePyramid* const __null__ = 0;

	reference.reset(__null__);
	current.reset(__null__);

	width = camera_info_msg->width;
	height = camera_info_msg->height;

	vis_->reset();
}

void CameraDenseTracker::handleConfig(dvo_tracking::CameraDenseTrackerConfig& config, uint32_t level)
{
  	TRACE()
	if(level == 0) 
	{
		return;
	}
  	config.run_dense_tracking = 1;
  	config.use_dense_tracking_estimate = 1;
  	// std::cout << "\tLevels: " << level << "\n";
  	// std::cout << "\tRun dense tracking: " << config.run_dense_tracking << "\n";
  	// std::cout << "\tUse dense tracking estimate: " << config.use_dense_tracking_estimate << "\n";
  	// std::cout << "\tTracker: " << tracker << "\n";
  	
	if(level & CameraDenseTracker_RunDenseTracking)
	{
		if(config.run_dense_tracking)
		{
			startSynchronizedImageStream();
		}
		else
		{
			stopSynchronizedImageStream();
			// force reset of tracker
			width = 0;
			height = 0;
		}
	}

	if(!config.run_dense_tracking && config.use_dense_tracking_estimate)
	{
		config.use_dense_tracking_estimate = false;
	}

	use_dense_tracking_estimate_ = config.use_dense_tracking_estimate;

	if(level & CameraDenseTracker_ConfigParam)
	{
		// fix config, so we don't die by accident
		if(config.coarsest_level < config.finest_level)
		{
			config.finest_level = config.coarsest_level;
		}

		dvo_tracking::util::updateConfigFromDynamicReconfigure(config, tracker_cfg);

		// we are called in the ctor as well, but at this point we don't have a tracker instance
		if(tracker)
		{
			// lock tracker so we don't reconfigure it while it is running
			boost::mutex::scoped_lock lock(tracker_mutex_);
			tracker->configure(tracker_cfg);
			camera->build(tracker_cfg.getNumLevels());
		}

		std::cout << "\n\tTRACKER CONFIGURATIONS: " << tracker_cfg << "\n";
	}


	if(level & CameraDenseTracker_MiscParam)
	{
		vis_->reset();
		delete vis_;

		if(config.reconstruction)
		{
			//vis_ = new dvo::visualization::PclCameraTrajectoryVisualizer();
			vis_ = new dvo_tracking::visualization::RosCameraTrajectoryVisualizer(nh_);
		}
		else
		{
			vis_ = new dvo::visualization::NoopCameraTrajectoryVisualizer();
		}
	}
}

void CameraDenseTracker::handlePose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose)
{
  	TRACE()
	tf::Transform tmp;

	tf::poseMsgToTF(pose->pose.pose, tmp);
	tf::transformTFToEigen(tmp, latest_absolute_transform_);
	// std::cout << "\t" << __LINE__ << "latest_absolute_transform_: \n" << latest_absolute_transform_.matrix() << "\n";
	if(!use_dense_tracking_estimate_)
		publishPose(pose->header, latest_absolute_transform_, "baselink_estimate");
}

void CameraDenseTracker::handleImages(
		const sensor_msgs::Image::ConstPtr& rgb_image_msg,
		const sensor_msgs::Image::ConstPtr& depth_image_msg,
		const sensor_msgs::CameraInfo::ConstPtr& rgb_camera_info_msg,
		const sensor_msgs::CameraInfo::ConstPtr& depth_camera_info_msg
)
{
  	TRACE()
	static stopwatch sw_callback("callback");
	sw_callback.start();

	// lock tracker so no one can reconfigure it
	boost::mutex::scoped_lock lock(tracker_mutex_);
	// VALGRIND_CHECK_VALUE_IS_DEFINED(rgb_camera_info_msg);
	// different size of rgb and depth image
	if(depth_camera_info_msg->width != rgb_camera_info_msg->width || depth_camera_info_msg->height != rgb_camera_info_msg->height)
	{
		ROS_WARN("RGB and depth image have different size!");

		return;
	}

	// something has changed
	if(hasChanged(rgb_camera_info_msg))
	{
		ROS_WARN("RGB image size has changed, resetting tracker!");

		reset(rgb_camera_info_msg);
	}

	cv::Mat intensity, depth;
	cv::Mat rgb_in = cv_bridge::toCvShare(rgb_image_msg)->image;

	if(rgb_in.channels() == 3)
	{
		cv::Mat tmp;
		cv::cvtColor(rgb_in, tmp, CV_BGR2GRAY, 1);

		tmp.convertTo(intensity, CV_32F);
	}
	else
	{
		rgb_in.convertTo(intensity, CV_32F);
	}

	cv::Mat depth_in = cv_bridge::toCvShare(depth_image_msg)->image;

	if(depth_in.type() == CV_16UC1)
	{
		SurfacePyramid::convertRawDepthImageSse(depth_in, depth, 0.001);
	}
	else
	{
		depth = depth_in;
	}
	reference.swap(current);
	current = camera->create(intensity, depth);

	// time delay compensation TODO: use driver settings instead
	std_msgs::Header h = rgb_image_msg->header;
	//h.stamp -= ros::Duration(0.05);

	static Eigen::Affine3d first;

	if(!reference)
	{
		accumulated_transform = latest_absolute_transform_ * from_baselink_to_kinect;
		// std::cout << "\nFIRST latest_absolute_transform_: \n" << accumulated_transform.matrix() << "\n\n";

		first = accumulated_transform;

		vis_->camera("first")->color(dvo::visualization::Color::blue()).update(current->level(0), accumulated_transform).show();

		return;
	}

	Eigen::Affine3d transform;

	static stopwatch sw_match("match", 100);
	sw_match.start();

	bool success = tracker->match(*reference, *current, transform);

	sw_match.stopAndPrint();
	// std::cout << "success: " << success << "\n";

	if(success)
	{
		frames_since_last_success = 0;
		accumulated_transform = accumulated_transform * transform;
		Eigen::Matrix<double, 6, 6> covariance;

		//tracker->getCovarianceEstimate(covariance);

		//std::cerr << covariance << std::endl << std::endl;

		vis_->trajectory("estimate")->color(dvo::visualization::Color::red()).add(accumulated_transform);
		vis_->camera("current")->color(dvo::visualization::Color::red()).update(current->level(0), accumulated_transform).show();
	}
	else
	{
		frames_since_last_success++;
		reference.swap(current);
		ROS_WARN("fail");
	}

	publishTransform(h, accumulated_transform * from_baselink_to_kinect.inverse(), "base_link_estimate");
//  publishTransform(rgb_image_msg->header, first_transform.inverse() * accumulated_transform, "asus_estimate");

	if(use_dense_tracking_estimate_)
	{
		publishPose(h, accumulated_transform * from_baselink_to_kinect.inverse(), "baselink_estimate");
	}

	sw_callback.stopAndPrint();
}

void CameraDenseTracker::publishTransform(const std_msgs::Header& header, const Eigen::Affine3d& transform, const std::string frame)
{
  	TRACE()
	static tf::TransformBroadcaster tb;

	tf::StampedTransform tf_transform;
	tf_transform.frame_id_ = "world";
	tf_transform.child_frame_id_ = frame;
	tf_transform.stamp_ = header.stamp;
	// std::cout << "\nPublished current transform/pose: \n" << transform.matrix() << "\n\n";

	tf::transformEigenToTF(transform, tf_transform);

	tb.sendTransform(tf_transform);
}

void CameraDenseTracker::publishPose(const std_msgs::Header& header, const Eigen::Affine3d& transform, const std::string frame)
{
  	TRACE()
	if(pose_pub_.getNumSubscribers() == 0) return;

	geometry_msgs::PoseWithCovarianceStampedPtr msg(new geometry_msgs::PoseWithCovarianceStamped);

	static int seq = 1;

	msg->header.seq = seq++;
	msg->header.frame_id = frame;
	msg->header.stamp = header.stamp;

	tf::Transform tmp;
	// std::cout << "\t"  << __LINE__ << "pose: \n" << transform.matrix() << "\n\n";

	tf::transformEigenToTF(transform, tmp);
	tf::poseTFToMsg(tmp, msg->pose.pose);

	msg->pose.covariance.assign(0.0);

	pose_pub_.publish(msg);
}

} /* namespace dvo_tracking */
