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

#include <dvo/visualization/object_trajectory_visualizer.h>
#line __LINE__ "object_trajectory_visualizer.cpp"

namespace dvo
{
namespace visualization
{

namespace internal
{

using namespace dvo::visualization;

class ObjectVisualizer : public CameraVisualizer
{
public:
	ObjectVisualizer(std::string name) :
		visibility_(ShowCamera),
		user_override_(false)
	{
  		TRACE()
		name_ = name;
	}

	virtual ~ObjectVisualizer()
	{
  		TRACE()
	};

	virtual void show(Option option = ShowCameraAndCloud)
	{
  		TRACE()
		if(!user_override_)
		{
			visibility_ = option;
		}
		updateVisualization();
	}

	virtual void hide()
	{
  		TRACE()
		show(ShowNothing);
	}

	virtual CameraVisualizer& onclick(const OnClickCallback& callback)
	{
  		TRACE()
		// onclick_callback_ = callback;
		return *this;
	}

	virtual CameraVisualizer& update(const dvo::core::RgbdImage& img, const Eigen::Affine3d& pose)
	{
  		TRACE()
  // 	Stored in img: intensity, intensity_dx, intensity_dy, depth, depth_dx, depth_dy;
  		d = img.depth.clone();
  		i = img.intensity.clone();

		// // tf::poseEigenToMsg(pose, marker_.pose);

		// visualization_msgs::InteractiveMarker tmp;
		// if(!marker_server_.get(name_, tmp) || hasColorChanged(tmp))
		// {
		// 	// std::cout << "updating marker...\n";
		// 	updateMarkerColor(marker_);
		// 	marker_server_.insert(marker_, marker_callback_);
		// }
		// else
		// {
		// 	// std::cout << "setting marker pose...\n";
		// 	marker_server_.setPose(name_, marker_.pose);
		// }
		// //originally commented out
		// marker_server_.applyChanges();
		// point_cloud_builder_.reset(new AsyncPointCloudBuilder::BuildJob(img, pose));

		return *this;
	}
private:
	Option visibility_;
	bool user_override_;
	cv::Mat d;
	cv::Mat i;
	void updateVisualization()
	{
		TRACE()
		switch(visibility_)
		{
			case ShowCameraAndCloud:
				// point_cloud_aggregator_.add(name(), boost::bind(&AsyncPointCloudBuilder::BuildJob::build, point_cloud_builder_));
				break;
			case ShowCamera:
				// point_cloud_aggregator_.remove(name());
				break;
			default:
				// marker_server_.erase(name_);
				// point_cloud_aggregator_.remove(name());
				break;
		}

	}
};

class CVTrajectoryVisualizer : public TrajectoryVisualizer
{
public:
	CVTrajectoryVisualizer(std::string& name)
	{
  		TRACE()
		createTrajectoryMarker(name);
	}

	virtual ~CVTrajectoryVisualizer()
	{
  		TRACE()
	};

	virtual TrajectoryVisualizer& add(const Eigen::Affine3d& pose)
	{
  		TRACE()
		// updateMarkerColor();
		geometry_msgs::Point p;
		p.x = pose.translation()(0);
		p.y = pose.translation()(1);
		p.z = pose.translation()(2);

		// marker_.controls[0].markers[0].points.push_back(p);

		// marker_server_.insert(marker_);
		// marker_server_.applyChanges();

		return *this;
	}
private:

	void createTrajectoryMarker(std::string& name)
	{
  		TRACE()
		// visualization_msgs::Marker m;
		// m.type = visualization_msgs::Marker::LINE_STRIP;
		// m.color.a = 1.0f;
		// m.scale.x = 0.02;

		// control.markers.push_back(m);

		// marker.header.frame_id = "/world";
		// marker.name = name + std::string("_trajectory");
		// marker.controls.push_back(control);
	}

	void updateMarkerColor()
	{
  		TRACE()
  // 		std::cout << "Trajectory Visualizer :: updating marker color...\n";
		// marker_.controls[0].markers[0].color.r = float(color().r);
		// marker_.controls[0].markers[0].color.g = float(color().g);
		// marker_.controls[0].markers[0].color.b = float(color().b);
	}
};

struct ObjectTrajectoryVisualizerImpl
{
	typedef std::map<std::string, CameraVisualizer::Ptr> CameraVisualizerMap;
	typedef std::map<std::string, TrajectoryVisualizer::Ptr> TrajectoryVisualizerMap;

	ObjectTrajectoryVisualizerImpl(ros::NodeHandle& nh) :
		nh_(nh)
	{
  		TRACE()
		// image_topic_ = it_.advertise("intensity_image", 1, true);
		// depth_topic_ = it_.advertise("depth_image", 1, true);
		// point_cloud_topic_ = nh_.advertise<AsyncPointCloudBuilder::PointCloud>("cloud", 1, true);
		update_timer_ = nh_.createTimer(ros::Duration(1.0), &ObjectTrajectoryVisualizerImpl::update, this, false, true);
	}

	~ObjectTrajectoryVisualizerImpl()
	{
  		TRACE()
	}

	CameraVisualizer::Ptr camera(std::string name)
	{
 		TRACE()
		CameraVisualizerMap::iterator camera = camera_visualizers_.find(name);
		if(camera_visualizers_.end() == camera)
		{
			camera = camera_visualizers_.insert(
					std::make_pair(name, CameraVisualizer::Ptr(new ObjectVisualizer(name)))
			).first;
		}
		return camera->second;
	}

	TrajectoryVisualizer::Ptr trajectory(std::string name)
	{
  		TRACE()
		TrajectoryVisualizerMap::iterator trajectory = trajectory_visualizers_.find(name);
		if(trajectory_visualizers_.end() == trajectory)
		{
			trajectory = trajectory_visualizers_.insert(
					std::make_pair(name, TrajectoryVisualizer::Ptr(new CVTrajectoryVisualizer(name)))
			).first;
		}

		return trajectory->second;
	}

	void reset()
	{
  		TRACE()
		camera_visualizers_.clear();
		trajectory_visualizers_.clear();
	}

	// interactive_markers::InteractiveMarkerServer* native()
	// {
 //  		TRACE()
	// 	return &marker_server_;
	// }
private:
	ros::NodeHandle& nh_;
	// image_transport::Publisher image_topic_;
	// ros::Publisher point_cloud_topic_;
	ros::Timer update_timer_;
	CameraVisualizerMap camera_visualizers_;
	TrajectoryVisualizerMap trajectory_visualizers_;

	void update(const ros::TimerEvent& e)
	{
 		// TRACE()
		// if(point_cloud_topic_.getNumSubscribers() == 0) return;
		// dvo::visualization::AsyncPointCloudBuilder::PointCloud::Ptr cloud = point_cloud_aggregator_.build();
		// // VALGRIND_CHECK_VALUE_IS_DEFINED(cloud);
		// cloud->header.frame_id = "/world";
		// cloud->is_dense = true;
		// point_cloud_topic_.publish(cloud);
	}
};

} /* namespace internal */

ObjectTrajectoryVisualizer::ObjectTrajectoryVisualizer(ros::NodeHandle& nh) :
		impl_(new internal::ObjectTrajectoryVisualizerImpl(nh))
{
	TRACE()
}

ObjectTrajectoryVisualizer::~ObjectTrajectoryVisualizer()
{
	TRACE()
	delete impl_;
}

dvo::visualization::CameraVisualizer::Ptr ObjectTrajectoryVisualizer::camera(std::string name)
{
	TRACE()
	return impl_->camera(name);
}

dvo::visualization::TrajectoryVisualizer::Ptr ObjectTrajectoryVisualizer::trajectory(std::string name)
{
	TRACE()
	return impl_->trajectory(name);
}

void ObjectTrajectoryVisualizer::reset()
{
	TRACE()
	impl_->reset();
}

bool ObjectTrajectoryVisualizer::native(void*& native_visualizer)
{
	TRACE()
	// native_visualizer = impl_->native();
	return true;
}

} /* namespace visualization */
} /* namespace dvo */
