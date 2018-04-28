#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/UInt32.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

sensor_msgs::CameraInfo depth_info;
sensor_msgs::CameraInfo rgb_info;


std::vector<double> U;
std::vector<double> V;
std::vector<Eigen::Vector3d> uv;
std::vector<Eigen::Vector3d> previous_uv;
std::vector<Eigen::Vector4d> p, ps;
std::vector<cv::Mat> image_buffer;
std::vector<cv::Mat> depth_buffer;
std::string window_name, roi_window;

Eigen::Vector4d T;
Eigen::Matrix3d R;
Eigen::Affine3d H;

cv::Mat display_frame;
geometry_msgs::Point roi_centroid;
geometry_msgs::Point roi_size;
int roi_height, roi_width;
int n_buffers, idx, height, width, buffer_id, init_count;
bool draw_trackers, new_image;
double fx;
double fy;
double cx;
double cy;

class Callbacks
{
public:
Callbacks()
{
	image_transport::ImageTransport it_(nh_);  
	sub_rgb = it_.subscribe("/camera/rgb/image_rect_color", 1, &Callbacks::rgbCallback, this);
	sub_depth = it_.subscribe("/camera/depth/image_rect", 1, &Callbacks::depthCallback, this);
	sub_rgb_info = nh_.subscribe("/camera/rgb/camera_info", 1, &Callbacks::rgbInfoCallback, this);
	sub_transform = nh_.subscribe("/object_pose", 1, &Callbacks::transformCallback, this);
	pub_size = nh_.advertise<geometry_msgs::Point>("/roi_size", 1);	
	pub_centroid = nh_.advertise<geometry_msgs::Point>("/roi_centroid", 1);	
}

void rgbCallback(const sensor_msgs::ImageConstPtr& msg)
{
	// new_image = false;
	if (!new_image)
	{
		try
		{
			cv::Mat tmp, roi;
			tmp = cv_bridge::toCvShare(msg, "bgr8")->image;
			if (roi_size.x != 0 && roi_size.y != 0)
			{
				pub_size.publish(roi_size);
				pub_centroid.publish(roi_centroid);
			}
			if (uv.size() == 4)
			{
				drawPatch(tmp);
			}
			cv::imshow(window_name, tmp);
			cv::waitKey(1);
			image_buffer[buffer_id] = tmp;
			new_image = true;
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
		}
	}
}

void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
	// std::cout << "new image: " << new_image << "\n";
	if (new_image)
	{
		try
		{
			// std::cout << "in\n"
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

void rgbInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& data)
{
	fx = data->P[0];
	fy = data->P[5];
	cx = data->P[2];
	cy = data->P[6];
	height = data->height;
	width = data->width;
}

void transformCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& data)
{	
	// std::cout << "received pose: " << data->pose.pose.position << "\n";
	while (init_count < 5)
	{	
		std::cout << "init count: " << init_count << "\n";
		init_count++;
		return;
	}
	T[0] = double(data->pose.pose.position.x);
	T[1] = double(data->pose.pose.position.y);
	T[2] = double(data->pose.pose.position.z);
	T[3] = 1;
	std::cout << "Translation: " << T << "\n";
	// Eigen::Quaterniond q(data->pose.pose.orientation.x, data->pose.pose.orientation.y, data->pose.pose.orientation.z, data->pose.pose.orientation.w);
	// R = q.toRotationMatrix();
	// std::cout << "Rotation: " << R.matrix() << "\n";
	calculateP();
}

void calculateUV()
{ // (7) calculate warped pixel coordinates
	for (int i = 0; i < uv.size(); i++)
	{
		previous_uv[i] = uv[i];
		uv[i][0] = ((fx * ps[i][0]) / ps[i][2]) + cx;
		uv[i][1] = ((fy * ps[i][1]) / ps[i][2]) + cy;
		std::cout << "fx: " << fx << "\n";
		std::cout << "fy: " << fy << "\n";
		std::cout << "cx: " << cx << "\n";
		std::cout << "cy: " << cy << "\n";
		std::cout << "ps: " << ps[i] << "\n";
		std::cout << "new uv point (" << uv[i][0] << ", " << uv[i][1] << ")\n";
	}
}

void calculateP()
{ // (4) 	T(g, p) = Rp + t 
	for (int i = 0; i < p.size(); i++)
	{
		Eigen::Vector4d tmp, P;
		P = p[i];
		p[i] = ps[i];

		// tmp = R*P;
		// std::cout << "\nP: \n" << P << "\n";
		// std::cout << "\nR*P: \n" << tmp << "\n";
		ps[i][0] = P[0] + T[0];
		ps[i][1] = P[1] + T[1];
		ps[i][2] = P[2] + T[2];
		std::cout << "\nNew 3D Point: \n" << ps[i] << "\n";
	}
	calculateUV();
}

void drawPatch(cv::Mat& frame) 
{
    line(frame, cv::Point(uv[0][0], uv[0][1]), cv::Point(uv[1][0], uv[1][1]), (255,255,255));
    line(frame, cv::Point(uv[1][0], uv[1][1]), cv::Point(uv[2][0], uv[2][1]), (255,255,255));
    line(frame, cv::Point(uv[2][0], uv[2][1]), cv::Point(uv[3][0], uv[3][1]), (255,255,255));
    line(frame, cv::Point(uv[3][0], uv[3][1]), cv::Point(uv[0][0], uv[0][1]), (255,255,255));
}

private:
	ros::NodeHandle nh_;
	image_transport::Subscriber sub_rgb;
	image_transport::Subscriber sub_depth;
	ros::Subscriber sub_rgb_info;
	ros::Subscriber sub_transform;
	ros::Publisher pub_size;
	ros::Publisher pub_centroid;
}; // end of Callbacks

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
		px = Z * ((u - cx) / fx);
		py = Z * ((v - cy) / fy);
		Eigen::Vector4d P(px, py, Z, 1);
		p[i] = P;
		std::cout << "fx: " << fx << "\n";
		std::cout << "fy: " << fy << "\n";
		std::cout << "cx: " << cx << "\n";
		std::cout << "cy: " << cy << "\n";
		std::cout << "P: " << P << "\n";
		std::cout << "u: " << u << "\n";
		std::cout << "v: " << v << "\n";
	}
}

void initializeCorners()
{
	Eigen::Vector3d v1, v2, v3, v4, l1, l2, intersection;
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
  	v1 << minu, minv, 1;
  	v2 << maxu, minv, 1;
  	v3 << maxu, maxv, 1;
  	v4 << minu, maxv, 1;
	// std::cout << "v1: " << v1.matrix() << " v2: " << v2.matrix() << "\nv3: " << v3.matrix() << " v4: " << v4.matrix() << "\n";
  	uv.push_back(v1);
	uv.push_back(v2);
	uv.push_back(v3);
	uv.push_back(v4);
  	l1 = v1.cross(v3);
  	l2 = v2.cross(v4);
  	intersection = l1.cross(l2);
  	roi_centroid.x = intersection(0) / intersection(2);
  	roi_centroid.y = intersection(1) / intersection(2);
  	roi_centroid.z = 1;
  	roi_size.x = int(maxu - minu);
  	roi_size.y = int(maxv - minv);
  	roi_size.z = 0;
  	initializeP();
}

void resetVariables()
{
    uv.clear();
    previous_uv.clear();
    p.clear();
    ps.clear();
    U.clear();
    V.clear();
	draw_trackers = false;
	new_image = false;
	n_buffers = 10;
	buffer_id = 0;
	idx = 0;
  	roi_centroid.x = 0;
  	roi_centroid.y = 0;
  	roi_centroid.z = 0;
  	roi_size.x = 0;
  	roi_size.y = 0;
  	roi_size.z = 0;
	height = 0;
	width = 0;
	fx = 0;
	fy = 0;
	cx = 0;
	cy = 0;	
}

void mouseHandler(int mouse_event, int x, int y, int flags, void* param) 
{
    if (mouse_event == CV_EVENT_RBUTTONUP) {
        resetVariables();
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
	resetVariables();
	p.resize(4);
	ps.resize(4);
	previous_uv.resize(4);
	init_count = 0;
	image_buffer.resize(n_buffers);
	depth_buffer.resize(n_buffers);
	for(int i = 0; i < n_buffers; i++)
	{
		image_buffer[i].create(height, width, CV_8UC3);
		depth_buffer[i].create(height, width, CV_8UC3);
	}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
  	Callbacks CBObject;
	initialize();
	ros::spin();
	cv::destroyWindow(window_name);
}