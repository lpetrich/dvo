
// #include <ros/ros.h>
// #include <ros/console.h>
// // #include <dynamic_reconfigure/server.h>
// #include <image_transport/image_transport.h>

// #include <boost/bind.hpp>
// #include <cv_bridge/cv_bridge.h>

// #include <dvo/dense_tracking.h>

// #include <dvo/core/intrinsic_matrix.h>
// #include <dvo/core/rgbd_image.h>
// #include <dvo/core/surface_pyramid.h>
// #include <dvo/core/macros.h>
// #include <dvo/core/assert.h>

// #include <dvo/util/stopwatch.h>

// #include <dvo_tracking/camera_dense_tracking.h>
// #include <dvo_tracking/util/util.h>
// #include <dvo_tracking/visualization/ros_camera_trajectory_visualizer.h>
// #include <dvo_tracking/camera_base.h>

// #include <dvo/visualization/camera_trajectory_visualizer.h>

// class DepthNode : public dvo_tracking::CameraBase
// {
// public:
// 	struct Config
// 	{
// 		bool EstimateTrajectory;
// 		std::string TrajectoryFile;
// 		std::string VideoFolder;
// 		std::string CameraFile;
// 		std::string RgbdPairFile;
// 		std::string GroundtruthFile;
// 		bool ShowEstimate;
// 		bool KeepAlive;
// 		bool EstimateRequired();
// 		bool VisualizationRequired();
// 	};
// 	DepthNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
// 	virtual ~DepthNode();

// 	virtual void handleImages(
// 			const sensor_msgs::Image::ConstPtr& rgb_image_msg,
// 			const sensor_msgs::Image::ConstPtr& depth_image_msg,
// 			const sensor_msgs::CameraInfo::ConstPtr& rgb_camera_info_msg,
// 			const sensor_msgs::CameraInfo::ConstPtr& depth_camera_info_msg
// 	);
// 	void run();
// 	bool configure();
// 	void createReferenceCamera(dvo::visualization::CameraTrajectoryVisualizerInterface* visualizer, const dvo::core::RgbdImage& img, const Eigen::Affine3d& pose);
// 	// void renderWhileSwitchAndNotTerminated(dvo::visualization::CameraTrajectoryVisualizerInterface* visualizer, const dvo::visualization::Switch& s);

// private:
// 	boost::shared_ptr<dvo::DenseTracker> tracker;
//   	Config cfg_;
// 	uint32_t width;
// 	uint32_t height;
// 	ros::NodeHandle &nh_, nh_vis_, &nh_private_;
// 	std::ostream *trajectory_out_;
// 	dvo::core::RgbdCameraPyramidPtr camera;
// 	dvo::core::RgbdImagePyramidPtr current, reference;
// 	dvo::visualization::CameraTrajectoryVisualizerInterface* visualizer;
// 	dvo::DenseTracker dense_tracker;
// 	dvo::DenseTracker::Config tracker_cfg;
// 	Eigen::Affine3d accumulated_transform, from_baselink_to_kinect, latest_absolute_transform_;
// 	Eigen::Affine3d trajectory, relative;

// 	size_t frames_since_last_success;
// 	// tf::TransformListener tl;
// 	// ros::Publisher pose_pub_;
// 	// ros::Subscriber pose_sub_;
// 	bool use_dense_tracking_estimate_;
// 	boost::mutex tracker_mutex_;

// 	bool hasChanged(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg);
// 	void reset(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg);
// };

// bool DepthNode::Config::EstimateRequired()
// {
//   return EstimateTrajectory || ShowEstimate;
// }

// bool DepthNode::Config::VisualizationRequired()
// {
//   return ShowEstimate;
// }

// DepthNode::DepthNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private) :
// 		CameraBase(nh, nh_private),
// 		nh_(nh),
// 		nh_vis_(nh, "dvo_vis"),
// 		nh_private_(nh_private),
// 		trajectory_out_(0),
// 		width(0),
// 		height(0)
// {
// 	TRACE()
// }

// DepthNode::~DepthNode()
// {
//   	TRACE()
// 	delete visualizer;
// }

// bool DepthNode::hasChanged(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg)
// {
//   	TRACE()

// 	return width != camera_info_msg->width || height != camera_info_msg->height;
// }

// void DepthNode::reset(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg)
// {
// 	std::cout << "resetting...\n";
// 	dvo::core::IntrinsicMatrix intrinsics = dvo::core::IntrinsicMatrix::create(camera_info_msg->P[0], camera_info_msg->P[5], camera_info_msg->P[2], camera_info_msg->P[6]);
// 	camera.reset(new dvo::core::RgbdCameraPyramid(camera_info_msg->width, camera_info_msg->height, intrinsics));
// 	camera->build(tracker_cfg.getNumLevels());
// 	tracker.reset(new dvo::DenseTracker(tracker_cfg));
// 	static dvo::core::RgbdImagePyramid* const __null__ = 0;
// 	reference.reset(__null__);
// 	current.reset(__null__);
// 	width = camera_info_msg->width;
// 	height = camera_info_msg->height;
// 	visualizer->reset();
// 	std::cout << "reset complete\n";
// }

// bool DepthNode::configure()
// {
//   	nh_private_.param("estimate_trajectory", cfg_.EstimateTrajectory, true);
//   	nh_private_.param("show_estimate", cfg_.ShowEstimate, true);
//   	nh_private_.param("keep_alive", cfg_.KeepAlive, cfg_.VisualizationRequired());
//   	std::cout << "\nshow estimate: " << cfg_.ShowEstimate << "\n";
// 	startSynchronizedImageStream();
// 	std::cout << "starting synchronized image stream...\n";
//   return true;
// }

// void DepthNode::handleImages(
// 		const sensor_msgs::Image::ConstPtr& rgb_image_msg,
// 		const sensor_msgs::Image::ConstPtr& depth_image_msg,
// 		const sensor_msgs::CameraInfo::ConstPtr& rgb_camera_info_msg,
// 		const sensor_msgs::CameraInfo::ConstPtr& depth_camera_info_msg
// )
// {
//   	TRACE()
//   	std::cout << "Images received!\n";
// 	boost::mutex::scoped_lock lock(tracker_mutex_);
// 	if(depth_camera_info_msg->width != rgb_camera_info_msg->width || depth_camera_info_msg->height != rgb_camera_info_msg->height)
// 	{
// 		ROS_WARN("RGB and depth image have different size!");
// 		return;
// 	}
// 	if(hasChanged(rgb_camera_info_msg))
// 	{
// 		ROS_WARN("RGB image size has changed, resetting tracker!");
// 		reset(rgb_camera_info_msg);
// 	}
// 	cv::Mat intensity, depth;
// 	cv::Mat rgb_in = cv_bridge::toCvShare(rgb_image_msg)->image;
// 	if(rgb_in.channels() == 3)
// 	{
// 		cv::Mat tmp;
// 		cv::cvtColor(rgb_in, tmp, CV_BGR2GRAY, 1);
// 		tmp.convertTo(intensity, CV_32F);
// 	}
// 	else
// 	{
// 		rgb_in.convertTo(intensity, CV_32F);
// 	}
// 	cv::Mat depth_in = cv_bridge::toCvShare(depth_image_msg)->image;
// 	if(depth_in.type() == CV_16UC1)
// 	{
// 		dvo::core::SurfacePyramid::convertRawDepthImageSse(depth_in, depth, 0.001);
// 	}
// 	else
// 	{
// 		depth = depth_in;
// 	}
// 	reference.swap(current);
// 	current = camera->create(intensity, depth);
// 	static Eigen::Affine3d first;
// 	if(!reference)
// 	{
// 		accumulated_transform = latest_absolute_transform_ * from_baselink_to_kinect;
// 		std::cout << "\t" << __LINE__ << "FIRST latest_absolute_transform_: \n" << accumulated_transform.matrix() << "\n\n";

// 		first = accumulated_transform;
// 		visualizer->camera("first")->color(dvo::visualization::Color::blue()).update(current->level(0), accumulated_transform).show();
// 		return;
// 	}
// 	Eigen::Affine3d transform;
// 	bool success = tracker->match(*reference, *current, transform);
// 	if(success)
// 	{
// 		frames_since_last_success = 0;
// 		accumulated_transform = accumulated_transform * transform;
// 		Eigen::Matrix<double, 6, 6> covariance;
// 		visualizer->trajectory("estimate")->color(dvo::visualization::Color::red()).add(accumulated_transform);
// 		visualizer->camera("current")->color(dvo::visualization::Color::red()).update(current->level(0), accumulated_transform).show();
// 	}
// 	else
// 	{
// 		frames_since_last_success++;
// 		reference.swap(current);
// 		ROS_WARN("fail");
// 	}
// }