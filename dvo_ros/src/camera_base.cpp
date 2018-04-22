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

#include <dvo_ros/camera_base.h>
#line __LINE__ "camera_base.cpp"

namespace dvo_ros
{

CameraBase::CameraBase(ros::NodeHandle& nh, ros::NodeHandle& nh_private) :
	nh_(nh),
	nh_private_(nh_private),

	rgb_image_subscriber_(nh, "/camera/rgb/image_rect_color", 30),
	depth_image_subscriber_(nh, "/camera/depth/image_rect", 30),
	rgb_camera_info_subscriber_(nh, "/camera/rgb/camera_info", 30),
	depth_camera_info_subscriber_(nh, "/camera/depth/camera_info", 30),

	synchronizer_(RGBDWithCameraInfoPolicy(5), rgb_image_subscriber_, depth_image_subscriber_, rgb_camera_info_subscriber_, depth_camera_info_subscriber_),

	connected(false)
{
	TRACE()
}

CameraBase::~CameraBase()
{
  	TRACE()
	stopSynchronizedImageStream();
}

bool CameraBase::isSynchronizedImageStreamRunning()
{
  	TRACE()
	return connected;
}

void CameraBase::startSynchronizedImageStream()
{
  	TRACE()
	if(!connected)
	{
		// std::cout << "synchronizing image streams...\n";
		connection = synchronizer_.registerCallback(boost::bind(&CameraBase::handleImages, this, _1, _2, _3, _4));
		connected = true;
	}
}

void CameraBase::stopSynchronizedImageStream()
{
  	TRACE()
	if(connected)
	{
		// std::cout << "disconnecting image streams...\n";
		connection.disconnect();
		connected = false;
	}
}

} /* namespace dvo_ros */
