#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Core>

std::vector<double> U;
std::vector<double> V;
std::vector<Eigen::Vector2d> uv;
std::vector<Eigen::Vector2d> previous_uv;
std::vector<Eigen::Vector3d> p;
std::vector<cv::Mat> image_buffer;
std::vector<cv::Mat> depth_buffer;
int n_buffers, idx, height, width, buffer_id;
std::string window_name;

cv::Mat display_frame;
bool draw_trackers, new_image;
double fx = 538.77;
double fy = 539.53;
double cx = 331.17;
double cy = 274.42;

void drawPatch(cv::Mat& frame) {
    line(frame, cv::Point(uv[0][0], uv[0][1]), cv::Point(uv[1][0], uv[1][1]), (255,255,255));
    line(frame, cv::Point(uv[1][0], uv[1][1]), cv::Point(uv[2][0], uv[2][1]), (255,255,255));
    line(frame, cv::Point(uv[2][0], uv[2][1]), cv::Point(uv[3][0], uv[3][1]), (255,255,255));
    line(frame, cv::Point(uv[3][0], uv[3][1]), cv::Point(uv[0][0], uv[0][1]), (255,255,255));
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	if (!new_image)
	{
		try
		{
			image_buffer[buffer_id] = cv_bridge::toCvShare(msg, "bgr8")->image;
			new_image = true;
			if (uv.size() == 4)
			{
				drawPatch(image_buffer[buffer_id]);
			}
			cv::imshow(window_name, image_buffer[buffer_id]);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
		}
	}
}

void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
	if (new_image)
	{
		try
		{
			depth_buffer[buffer_id] = cv_bridge::toCvShare(msg, "32FC1")->image;
			new_image = false;
			idx = buffer_id;
			buffer_id = (buffer_id + 1) % n_buffers;
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("Could not convert from '%s' to '32FC1'.", msg->encoding.c_str());
		}
	}
}

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& data)
{
	std::cout << "received pose: " << data->pose.pose.position << "\n";

	// R = 0;
	// t = 0;

}

void warpedCoords()
{
	// std::cout << "depth at trackers:\n" << image_buffer[buffer_id].at<double>(uv[0][0], uv[0][1]) << "\n";
	double px, py, u, v, Z;
	for (int i = 0; i < uv.size(); i++)
	{	
		u = uv[i][0];
		v = uv[i][1];
		Z = image_buffer[buffer_id].at<double>(u, v) * 0.001;
		px = Z * ((u + cx) / fx);
		py = Z * ((v + cy) / fy);
		Eigen::Vector3d P(px, py, Z);
		p.push_back(P);
		std::cout << "P: " << P << "\n";
	}
}

void initializeP()
{
	double px, py, u, v, Z;
	for (int i = 0; i < uv.size(); i++)
	{	
		u = uv[i][0];
		v = uv[i][1];
		Z = depth_buffer[buffer_id].at<double>(u, v);
		if (!std::isfinite(Z))
		{	
			std::cout << "z is nan, setting to 1...\n";
			Z = 1.0;
		}
		px = Z * ((u + cx) / fx);
		py = Z * ((v + cy) / fy);
		Eigen::Vector3d P(px, py, Z);
		p.push_back(P);
		std::cout << "P: " << P << "\n";
	}
}

void initializeCorners()
{
	double minu, maxu, minv, maxv;
	if (U[0] < U[1]) 
	{
		minu = U[0];
		maxu = U[1];
	}
	else
	{
		minu = U[1];
		maxu = U[0];
	}
	if (V[0] < V[1])
	{
		minv = V[0];
		maxv = V[1];
	}
	else
	{
		minv = V[1];
		maxv = V[0];
	}
	Eigen::Vector2d c1(minu, minv);
	Eigen::Vector2d c2(maxu, minv);
	Eigen::Vector2d c3(maxu, maxv);
	Eigen::Vector2d c4(minu, maxv);
	uv.push_back(c1);
	uv.push_back(c2);
	uv.push_back(c3);
	uv.push_back(c4);
}

void mouseHandler(int mouse_event, int x, int y, int flags, void* param) 
{
    // Right mouse click restarts setting of tracker points
    if (mouse_event == CV_EVENT_RBUTTONUP) {
        uv.clear();
        previous_uv.clear();
        p.clear();
        U.clear();
        V.clear();
        new_image = false;
        draw_trackers = false;
        return;
    }
    if (mouse_event == CV_EVENT_LBUTTONUP && U.size() < 2) 
    {
        ROS_DEBUG_STREAM("Click at x: " << x << ", " << y);
        U.push_back(x);
        V.push_back(y);
        if (U.size() == 2) 
        {	
        	initializeCorners();
    		draw_trackers = true;
        }
        return;
    }
}

void initialize()
{
	window_name = "results";
	cv::namedWindow(window_name);
	cv::setMouseCallback(window_name, mouseHandler);
	cv::startWindowThread();
	draw_trackers = false;
	new_image = false;
	n_buffers = 10;
	buffer_id = 0;
	height = 480;
	width = 640;
	depth_buffer.resize(n_buffers);
	image_buffer.resize(n_buffers);
	for(int i = 0; i < n_buffers; i++)
	{
		image_buffer[i].create(height, width, CV_8UC3);
		depth_buffer[i].create(height, width, CV_8UC3);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	initialize();
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber img_sub = it.subscribe("/camera/rgb/image_rect_color", 1, imageCallback);
	image_transport::Subscriber depth_sub = it.subscribe("/camera/depth/image_rect", 1, depthCallback);
	ros::Subscriber pose_sub = nh.subscribe("/object_pose", 1, poseCallback);
	ros::spin();
	cv::destroyWindow(window_name);
}