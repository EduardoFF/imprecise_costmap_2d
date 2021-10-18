/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <imprecise_costmap_2d/imprecise_costmap_2d.h>
#include <costmap_2d/footprint.h>
#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>

using std::vector;

namespace imprecise_costmap_2d
{

  ImpreciseCostmap::ImpreciseCostmap(std::string global_frame, costmap_2d::Costmap2D *p_costmap, bool rolling_window, bool track_unknown) :
    costmap_(),
    low_costmap_(),
    high_costmap_(),
    p_costmap_(p_costmap),
    global_frame_(global_frame),
    rolling_window_(rolling_window),
    current_(false),
    minx_(0.0),
    miny_(0.0),
    maxx_(0.0),
    maxy_(0.0),
    bx0_(0),
    bxn_(0),
    by0_(0),
    byn_(0),
    initialized_(false),
    size_locked_(false),
    circumscribed_radius_(1.0),
    inscribed_radius_(0.1)
{
  if (track_unknown)
    {
      low_costmap_.setDefaultValue(costmap_2d::NO_INFORMATION);
      high_costmap_.setDefaultValue(costmap_2d::NO_INFORMATION);
      costmap_.setDefaultValue(costmap_2d::NO_INFORMATION);
      printf("Tracking unknown");
      ROS_INFO("Tracking unknown");
    }
  else
    {
      ROS_INFO("NOT Tracking unknown");
      costmap_.setDefaultValue(costmap_2d::FREE_SPACE);
    }
  
  ROS_INFO("Initialized Imprecise Costmap\n");

}

ImpreciseCostmap::~ImpreciseCostmap()
{

}

void ImpreciseCostmap::resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x,
                               double origin_y, bool size_locked)
{
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_.getMutex()));
  ROS_INFO("Resizing maps to %d %d res: %.2f, orig (%.2f, %.2f) size locked? %d\n", size_x, size_y, resolution, origin_x, origin_y, size_locked);
  size_locked_ = size_locked;
  costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);

  low_costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  high_costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  input_costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);

  /*
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    (*plugin)->matchSize();
  }
  */
}

void ImpreciseCostmap::updateMap(double robot_x, double robot_y, double robot_yaw)
{
  ROS_INFO("Updating costmap (%.2f, %.2f):%.2f", robot_x, robot_y,  robot_yaw);
  // Lock for the remainder of this function, some plugins (e.g. VoxelLayer)
  // implement thread unsafe updateBounds() functions.
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_.getMutex()));

  // resize costmap if size, resolution or origin do not match
  costmap_2d::Costmap2D* p_map = p_costmap_;
  costmap_2d::Costmap2D* master = &costmap_;
  unsigned char *low_array = low_costmap_.getCharMap();
  unsigned char *high_array = low_costmap_.getCharMap();
  unsigned char *p_array = p_costmap_->getCharMap();
  minx_ = miny_ = -1e30;
  maxx_ = maxy_ = 1e30;
  
  int x0, xn, y0, yn;
  master->worldToMapEnforceBounds(minx_, miny_, x0, y0);
  master->worldToMapEnforceBounds(maxx_, maxy_, xn, yn);
  
  if (!isRolling() && (p_map->getSizeInCellsX() != master->getSizeInCellsX() ||
		       p_map->getSizeInCellsY() != master->getSizeInCellsY() ||
		       p_map->getResolution() != master->getResolution() ||
		       p_map->getOriginX() != master->getOriginX() ||
		       p_map->getOriginY() != master->getOriginY()))
    {
      // Update the size of the layered costmap (and all layers, including this one)
      ROS_INFO("Resizing costmaps to %d X %d at %f m/pix",
	       p_map->getSizeInCellsX(), p_map->getSizeInCellsY(), p_map->getResolution());
      resizeMap(p_map->getSizeInCellsX(),
		p_map->getSizeInCellsY(),
		p_map->getResolution(),
		p_map->getOriginX(),
		p_map->getOriginY(), true);
      
    }
  
  


  
  // if we're using a rolling buffer costmap... we need to update the origin using the robot's position
  /* ROLLING NOT SUPPORTED
  if (isRolling())
  {
    double new_origin_x = robot_x - costmap_.getSizeInMetersX() / 2;
    double new_origin_y = robot_y - costmap_.getSizeInMetersY() / 2;
    costmap_.updateOrigin(new_origin_x, new_origin_y);
  }
  */

  // minx miny maxx maxy must be updated
  // call updateBounds?
  /*
  x0 = std::max(0, x0);
  xn = std::min(int(costmap_.getSizeInCellsX()), xn + 1);
  y0 = std::max(0, y0);
  yn = std::min(int(costmap_.getSizeInCellsY()), yn + 1);
  */

  ROS_INFO("Updating area x: [%d, %d] y: [%d, %d] map size %d x %d (%.2f x %.2f)",
	    x0, xn, y0, yn, costmap_.getSizeInCellsX(), costmap_.getSizeInCellsY(),costmap_.getSizeInMetersX(), costmap_.getSizeInMetersY());

  //  if (xn < x0 || yn < y0)
  //  return;

  unsigned char* master_array = costmap_.getCharMap();
  unsigned int span = costmap_.getSizeInCellsX();
  int min_i = x0;
  int min_j = y0;
  int max_i = xn;
  int max_j = yn;
  ROS_INFO("Filling %d %d %d %d\n", min_i, max_i, min_j, max_j);

  //  low_costmap_.resetMap(min_i,min_j,max_i,max_j);
  //high_costmap_.resetMap(min_i,min_j,max_i,max_j);

  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      double l_cost = low_costmap_.getCost(i,j);
      double h_cost = high_costmap_.getCost(i,j);
      double s_cost = p_map->getCost(i,j);
      double pp = costmap_2d::NO_INFORMATION;
      if( l_cost == costmap_2d::NO_INFORMATION
	  && h_cost == costmap_2d::NO_INFORMATION)
	{
	  /// just take the sensor map
	  pp = s_cost;
	}
      else
	{
	  pp = 1.0*l_cost*(1./100.)*(1.0 - (1./100.)*(h_cost - l_cost));
	  if( s_cost != costmap_2d::NO_INFORMATION )
	    pp = fmax(pp, s_cost);
	  pp = fmin(costmap_2d::LETHAL_OBSTACLE,
		    ceil((costmap_2d::LETHAL_OBSTACLE)*pp));
	  pp = fmax(pp, 0);
	}
      unsigned char c = (unsigned char) pp ;
      costmap_.setCost(i,j,c);
      }
  }      
  

  //  costmap_.resetMap(x0, y0, xn, yn);
  int update_cnt = 0;
  /*
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin)
  {
    ROS_DEBUG("updateCosts layer %s (%d)", (*plugin)->getName().c_str(), update_cnt++);
    (*plugin)->updateCosts(costmap_, x0, y0, xn, yn);
  }
  */
  ROS_DEBUG("end Updating costs");

  bx0_ = x0;
  bxn_ = xn;
  by0_ = y0;
  byn_ = yn;

  initialized_ = true;
}

bool ImpreciseCostmap::isCurrent()
{
  current_ = true;
  return current_;
}

void ImpreciseCostmap::setFootprint(const std::vector<geometry_msgs::Point>& footprint_spec)
{
  footprint_ = footprint_spec;
  costmap_2d::calculateMinAndMaxDistances(footprint_spec, inscribed_radius_, circumscribed_radius_);

}

}  // namespace costmap_2d
