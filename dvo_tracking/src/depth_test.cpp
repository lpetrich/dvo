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
#include <dvo_tracking/depth_test.h>

namespace dvo_tracking
{

using namespace dvo;
using namespace dvo::core;
using namespace dvo::util;

DepthNode::DepthNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private) :
	CameraBase(nh, nh_private),
	tracker_cfg(DenseTracker::getDefaultConfig()),
	frames_since_last_success(0),
	reconfigure_server_(nh_private),
	vis_(new dvo::visualization::NoopCameraTrajectoryVisualizer()),
	use_dense_tracking_estimate_(false),
	width(0),
	height(0)
{
  	TRACE()
	ReconfigureServer::CallbackType reconfigure_server_callback = boost::bind(&DepthNode::handleConfig, this, _1, _2);
	reconfigure_server_.setCallback(reconfigure_server_callback);
	cv::namedWindow("Depth Tracking", cv::WINDOW_AUTOSIZE);
	pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/object_pose", 1);
	dvo_tracking::util::tryGetTransform(from_baselink_to_kinect, tl, "/camera_link", "/camera_rgb_optical_frame");
	latest_absolute_transform_.setIdentity();
	accumulated_transform.setIdentity();
}

DepthNode::~DepthNode()
{
  	TRACE()
	delete vis_;
}

bool DepthNode::hasChanged(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg)
{
  	TRACE()
	return width != camera_info_msg->width || height != camera_info_msg->height;
}

void DepthNode::reset(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg)
{
  	TRACE()
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

void DepthNode::handleConfig(dvo_tracking::CameraDenseTrackerConfig& config, uint32_t level)
{
  	TRACE()
	if(level == 0) 
	{
		return;
	}
  	config.run_dense_tracking = 1;
  	config.use_dense_tracking_estimate = 1;
	if(level & CameraDenseTracker_RunDenseTracking)
	{
		if(config.run_dense_tracking)
		{
			startSynchronizedImageStream();
		}
		else
		{
			stopSynchronizedImageStream();
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
		if(config.coarsest_level < config.finest_level)
		{
			config.finest_level = config.coarsest_level;
		}
		dvo_tracking::util::updateConfigFromDynamicReconfigure(config, tracker_cfg);
		if(tracker)
		{
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
		vis_ = new dvo::visualization::NoopCameraTrajectoryVisualizer();
	}
}

void DepthNode::handleImages(
		const sensor_msgs::Image::ConstPtr& rgb_image_msg,
		const sensor_msgs::Image::ConstPtr& depth_image_msg,
		const sensor_msgs::CameraInfo::ConstPtr& rgb_camera_info_msg,
		const sensor_msgs::CameraInfo::ConstPtr& depth_camera_info_msg
)
{
  	TRACE()
	boost::mutex::scoped_lock lock(tracker_mutex_);
	if(depth_camera_info_msg->width != rgb_camera_info_msg->width || depth_camera_info_msg->height != rgb_camera_info_msg->height)
	{
		ROS_WARN("RGB and depth image have different size!");
		return;
	}
	if(hasChanged(rgb_camera_info_msg))
	{
		ROS_WARN("RGB image size has changed, resetting tracker!");
		reset(rgb_camera_info_msg);
	}
	cv::Mat intensity, depth;
	cv::Mat rgb_in = cv_bridge::toCvShare(rgb_image_msg)->image;
	current_view = rgb_in.clone();
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
	std_msgs::Header h = rgb_image_msg->header;
	static Eigen::Affine3d first;
	if(!reference)
	{
		accumulated_transform = latest_absolute_transform_ * from_baselink_to_kinect;
		first = accumulated_transform;
		vis_->camera("first")->color(dvo::visualization::Color::blue()).update(current->level(0), accumulated_transform).show();
		return;
	}
	Eigen::Affine3d transform;
	bool success = tracker->match(*reference, *current, transform);
	if(success)
	{
		frames_since_last_success = 0;
		accumulated_transform = accumulated_transform * transform;
		vis_->trajectory("estimate")->color(dvo::visualization::Color::red()).add(accumulated_transform);
		vis_->camera("current")->color(dvo::visualization::Color::red()).update(current->level(0), accumulated_transform).show();
	}
	else
	{
		frames_since_last_success++;
		reference.swap(current);
		ROS_WARN("fail");
	}
	publishPose(h, accumulated_transform * from_baselink_to_kinect.inverse(), "object_estimate");
}

void DepthNode::publishPose(const std_msgs::Header& header, const Eigen::Affine3d& transform, const std::string frame)
{
  	TRACE()
	geometry_msgs::PoseWithCovarianceStampedPtr msg(new geometry_msgs::PoseWithCovarianceStamped);
	static int seq = 1;
	msg->header.seq = seq++;
	msg->header.frame_id = frame;
	msg->header.stamp = header.stamp;
	tf::Transform tmp;
	tf::transformEigenToTF(transform, tmp);
	tf::poseTFToMsg(tmp, msg->pose.pose);
	msg->pose.covariance.assign(0.0);
	pose_pub_.publish(msg);
}

} /* namespace dvo_tracking */

int main(int argc, char **argv)
{
	TRACE()
	ros::init(argc, argv, "depth_test", ros::init_options::AnonymousName);
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	dvo_tracking::DepthNode depthnode(nh, nh_private);
	ros::spin();
	return 0;
}

