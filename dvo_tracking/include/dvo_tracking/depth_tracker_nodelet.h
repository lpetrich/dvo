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

#ifndef DEPTH_TRACKER_NODELET_H_
#define DEPTH_TRACKER_NODELET_H_

#include <nodelet/nodelet.h>

#include <dvo_tracking/camera_dense_tracking.h>
#include <dvo/core/macros.h>

namespace dvo_tracking
{

class DepthTrackerNodelet : public nodelet::Nodelet
{
private:
  std::auto_ptr<dvo_tracking::CameraDenseTracker> tracker_;
public:
  DepthTrackerNodelet();
  virtual ~DepthTrackerNodelet();

  virtual void onInit();
};

} /* namespace dvo_tracking */
#endif /* DEPTH_TRACKER_NODELET_H_ */
