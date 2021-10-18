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
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/footprint.h>
#include <imprecise_costmap_2d/imprecise_costmap_2d_ros.h>
#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <imprecise_costmap_2d/eigenmvn.h>
#include <Eigen/Dense>
#include <random>

#include <boost/random.hpp>
using namespace std;

namespace imprecise_costmap_2d
{

  /**
  Take a pair of un-correlated variances.
  Create a covariance matrix by correlating 
  them, sandwiching them in a rotation matrix.
*/
Eigen::Matrix2d genCovar(double v0,double v1,double theta)
{
  Eigen::Matrix2d rot = Eigen::Rotation2Dd(theta).matrix();
  return rot*Eigen::DiagonalMatrix<double,2,2>(v0,v1)*rot.transpose();
}


void move_parameter(ros::NodeHandle& old_h, ros::NodeHandle& new_h, std::string name, bool should_delete = true)
{
  if (!old_h.hasParam(name))
    return;

  XmlRpc::XmlRpcValue value;
  old_h.getParam(name, value);
  new_h.setParam(name, value);
  if (should_delete) old_h.deleteParam(name);
}

ImpreciseCostmap2DROS::ImpreciseCostmap2DROS(const std::string& name, tf2_ros::Buffer& tf) :
    i_costmap_(NULL),
    s_costmap_(NULL),
    name_(name),
    tf_(tf),
    transform_tolerance_(0.3),
    map_update_thread_shutdown_(false),
    stop_updates_(false),
    initialized_(true),
    stopped_(false),
    robot_stopped_(false),
    map_update_thread_(NULL),
    last_publish_(0),
    plugin_loader_("costmap_2d", "costmap_2d::Layer"),
    publisher_(NULL),
    dsrv_(NULL),
    footprint_padding_(0.0)
{
  // Initialize old pose with something
  tf2::toMsg(tf2::Transform::getIdentity(), old_pose_.pose);

  ros::NodeHandle private_nh("~/" + name);
  ros::NodeHandle g_nh;

  // get two frames
  private_nh.param("global_frame", global_frame_, std::string("map"));
  private_nh.param("robot_base_frame", robot_base_frame_, std::string("base_link"));

  ros::Time last_error = ros::Time::now();
  std::string tf_error;
  // we need to make sure that the transform between the robot base frame and the global frame is available
  while (ros::ok()
      && !tf_.canTransform(global_frame_, robot_base_frame_, ros::Time(), ros::Duration(0.1), &tf_error))
  {
    ros::spinOnce();
    if (last_error + ros::Duration(5.0) < ros::Time::now())
    {
      ROS_WARN("Timed out waiting for transform from %s to %s to become available before running costmap, tf error: %s",
               robot_base_frame_.c_str(), global_frame_.c_str(), tf_error.c_str());
      last_error = ros::Time::now();
    }
    // The error string will accumulate and errors will typically be the same, so the last
    // will do for the warning above. Reset the string here to avoid accumulation.
    tf_error.clear();
  }

  // check if we want a rolling window version of the costmap
  bool rolling_window, track_unknown_space, always_send_full_costmap;
  private_nh.param("rolling_window", rolling_window, false);
  private_nh.param("track_unknown_space", track_unknown_space, true);
  private_nh.param("always_send_full_costmap", always_send_full_costmap, false);

  std::string map_topic;
  if (!private_nh.searchParam("map_topic", map_topic))
  {
    printf("map_topic not found\n");
    map_topic = "/orig_map";
  }

  //private_nh.param("map_topic", map_topic, "standard_global_costmap");
  //TODO other params of static layer

  // manage the params of static layer
  ros::NodeHandle map_layer(private_nh, "static_layer");
  move_parameter(private_nh, map_layer, "map_topic");
  move_parameter(private_nh, map_layer, "unknown_cost_value");
  move_parameter(private_nh, map_layer, "lethal_cost_threshold");
  move_parameter(private_nh, map_layer, "track_unknown_space");


  s_costmap_ = new costmap_2d::LayeredCostmap(global_frame_, rolling_window, track_unknown_space);

  i_costmap_ = new ImpreciseCostmap(global_frame_, s_costmap_->getCostmap(), rolling_window, track_unknown_space);


  boost::shared_ptr<costmap_2d::Layer> plugin =
    plugin_loader_.createInstance("costmap_2d::StaticLayer");
  s_costmap_->addPlugin(plugin);
  plugin->initialize(s_costmap_, name + "/static_layer", &tf_);


  // subscribe to the footprint topic
  std::string topic_param, topic;
  if (!private_nh.searchParam("footprint_topic", topic_param))
  {
    topic_param = "footprint_topic";
  }

  private_nh.param(topic_param, topic, std::string("footprint"));
  footprint_sub_ = private_nh.subscribe(topic, 1, &ImpreciseCostmap2DROS::setUnpaddedRobotFootprintPolygon, this);

  hinput_sub_ = private_nh.subscribe("human_input", 100, &ImpreciseCostmap2DROS::incomingInput, this);

  
  if (!private_nh.searchParam("published_footprint_topic", topic_param))
  {
    topic_param = "published_footprint";
  }

  private_nh.param(topic_param, topic, std::string("oriented_footprint"));
  footprint_pub_ = private_nh.advertise<geometry_msgs::PolygonStamped>("footprint", 1);

  setUnpaddedRobotFootprint(costmap_2d::makeFootprintFromParams(private_nh));

  ROS_INFO("always_send_full_costmap %d\n", always_send_full_costmap);
  publisher_ = new costmap_2d::Costmap2DPublisher(&private_nh, i_costmap_->getCostmap(), global_frame_, "imprecise_costmap",
                                      always_send_full_costmap);

  scmap_publisher_ = new costmap_2d::Costmap2DPublisher(&private_nh, s_costmap_->getCostmap(), global_frame_, "sensor_costmap",
                                      always_send_full_costmap);

  
  low_cmap_publisher_ = new costmap_2d::Costmap2DPublisher(&private_nh, i_costmap_->getLowCostmap(), global_frame_, "imprecise_costmap_low",
                                      always_send_full_costmap);

  high_cmap_publisher_ = new costmap_2d::Costmap2DPublisher(&private_nh, i_costmap_->getHighCostmap(), global_frame_, "imprecise_costmap_high",
                                      always_send_full_costmap);

  // create a thread to handle updating the map
  stop_updates_ = false;
  initialized_ = true;
  stopped_ = false;

  // Create a time r to check if the robot is moving
  robot_stopped_ = false;
  timer_ = private_nh.createTimer(ros::Duration(.1), &ImpreciseCostmap2DROS::movementCB, this);

  dsrv_ = new dynamic_reconfigure::Server<ImpreciseCostmap2DConfig>(ros::NodeHandle("~/" + name));
  dynamic_reconfigure::Server<ImpreciseCostmap2DConfig>::CallbackType cb = boost::bind(&ImpreciseCostmap2DROS::reconfigureCB, this, _1,
                                                                              _2);
  dsrv_->setCallback(cb);
}

void ImpreciseCostmap2DROS::setUnpaddedRobotFootprintPolygon(const geometry_msgs::Polygon& footprint)
{
  setUnpaddedRobotFootprint(costmap_2d::toPointVector(footprint));
}

ImpreciseCostmap2DROS::~ImpreciseCostmap2DROS()
{
  map_update_thread_shutdown_ = true;
  if (map_update_thread_ != NULL)
  {
    map_update_thread_->join();
    delete map_update_thread_;
  }
  if (publisher_ != NULL)
    delete publisher_;

  if (scmap_publisher_ != NULL)
    delete scmap_publisher_;

  if (low_cmap_publisher_ != NULL)
    delete low_cmap_publisher_;

  if (high_cmap_publisher_ != NULL)
    delete high_cmap_publisher_;

  delete i_costmap_;
  delete s_costmap_;
  delete dsrv_;
}
  /*
void ImpreciseCostmap2DROS::resetOldParameters(ros::NodeHandle& nh)
{
 
  ROS_INFO("Loading from pre-hydro parameter style");
  bool flag;
  std::string s;
  std::vector < XmlRpc::XmlRpcValue > plugins;

  XmlRpc::XmlRpcValue::ValueStruct map;
  SuperValue super_map;
  SuperValue super_array;

  if (nh.getParam("static_map", flag) && flag)
  {
    map["name"] = XmlRpc::XmlRpcValue("static_layer");
    map["type"] = XmlRpc::XmlRpcValue("costmap_2d::StaticLayer");
    super_map.setStruct(&map);
    plugins.push_back(super_map);

    ros::NodeHandle map_layer(nh, "static_layer");
    move_parameter(nh, map_layer, "map_topic");
    move_parameter(nh, map_layer, "unknown_cost_value");
    move_parameter(nh, map_layer, "lethal_cost_threshold");
    move_parameter(nh, map_layer, "track_unknown_space", false);
  }

  ros::NodeHandle obstacles(nh, "obstacle_layer");
  if (nh.getParam("map_type", s) && s == "voxel")
  {
    map["name"] = XmlRpc::XmlRpcValue("obstacle_layer");
    map["type"] = XmlRpc::XmlRpcValue("costmap_2d::VoxelLayer");
    super_map.setStruct(&map);
    plugins.push_back(super_map);

    move_parameter(nh, obstacles, "origin_z");
    move_parameter(nh, obstacles, "z_resolution");
    move_parameter(nh, obstacles, "z_voxels");
    move_parameter(nh, obstacles, "mark_threshold");
    move_parameter(nh, obstacles, "unknown_threshold");
    move_parameter(nh, obstacles, "publish_voxel_map");
  }
  else
  {
    map["name"] = XmlRpc::XmlRpcValue("obstacle_layer");
    map["type"] = XmlRpc::XmlRpcValue("costmap_2d::ObstacleLayer");
    super_map.setStruct(&map);
    plugins.push_back(super_map);
  }

  move_parameter(nh, obstacles, "max_obstacle_height");
  move_parameter(nh, obstacles, "raytrace_range");
  move_parameter(nh, obstacles, "obstacle_range");
  move_parameter(nh, obstacles, "track_unknown_space", true);
  nh.param("observation_sources", s, std::string(""));
  std::stringstream ss(s);
  std::string source;
  while (ss >> source)
  {
    move_parameter(nh, obstacles, source);
  }
  move_parameter(nh, obstacles, "observation_sources");

  ros::NodeHandle inflation(nh, "inflation_layer");
  move_parameter(nh, inflation, "cost_scaling_factor");
  move_parameter(nh, inflation, "inflation_radius");
  map["name"] = XmlRpc::XmlRpcValue("inflation_layer");
  map["type"] = XmlRpc::XmlRpcValue("costmap_2d::InflationLayer");
  super_map.setStruct(&map);
  plugins.push_back(super_map);

  super_array.setArray(&plugins);
  nh.setParam("plugins", super_array);
}
  */

void ImpreciseCostmap2DROS::reconfigureCB(imprecise_costmap_2d::ImpreciseCostmap2DConfig &config, uint32_t level)
{
  transform_tolerance_ = config.transform_tolerance;
  if (map_update_thread_ != NULL)
  {
    map_update_thread_shutdown_ = true;
    map_update_thread_->join();
    ROS_INFO("Deleted map_update_thread\n");
    delete map_update_thread_;
  }
  map_update_thread_shutdown_ = false;
  double map_update_frequency = config.update_frequency;

  double map_publish_frequency = config.publish_frequency;
  if (map_publish_frequency > 0)
    publish_cycle = ros::Duration(1 / map_publish_frequency);
  else
    publish_cycle = ros::Duration(-1);
  ROS_INFO("map_publish_frequency %.2f, publish_cycle %f\n", map_publish_frequency, publish_cycle.toSec());

  // find size parameters
  double map_width_meters = config.width, map_height_meters = config.height, resolution = config.resolution, origin_x =
             config.origin_x,
         origin_y = config.origin_y;

  if (!i_costmap_->isSizeLocked())
  {
    i_costmap_->resizeMap((unsigned int)(map_width_meters / resolution),
                                (unsigned int)(map_height_meters / resolution), resolution, origin_x, origin_y);
  }

  // If the padding has changed, call setUnpaddedRobotFootprint() to
  // re-apply the padding.
  if (footprint_padding_ != config.footprint_padding)
  {
    footprint_padding_ = config.footprint_padding;
    setUnpaddedRobotFootprint(unpadded_footprint_);
  }

  readFootprintFromConfig(config, old_config_);

  old_config_ = config;

  ROS_INFO("Update frequency %.2f\n", map_update_frequency);
  map_update_thread_ = new boost::thread(boost::bind(&ImpreciseCostmap2DROS::mapUpdateLoop, this, map_update_frequency));
}

void ImpreciseCostmap2DROS::readFootprintFromConfig(const imprecise_costmap_2d::ImpreciseCostmap2DConfig &new_config,
                                           const imprecise_costmap_2d::ImpreciseCostmap2DConfig &old_config)
{
  // Only change the footprint if footprint or robot_radius has
  // changed.  Otherwise we might overwrite a footprint sent on a
  // topic by a dynamic_reconfigure call which was setting some other
  // variable.
  if (new_config.footprint == old_config.footprint &&
      new_config.robot_radius == old_config.robot_radius)
  {
    return;
  }

  if (new_config.footprint != "" && new_config.footprint != "[]")
  {
    std::vector<geometry_msgs::Point> new_footprint;
    if (costmap_2d::makeFootprintFromString(new_config.footprint, new_footprint))
    {
        setUnpaddedRobotFootprint(new_footprint);
    }
    else
    {
        ROS_ERROR("Invalid footprint string from dynamic reconfigure");
    }
  }
  else
  {
    // robot_radius may be 0, but that must be intended at this point.
    setUnpaddedRobotFootprint(costmap_2d::makeFootprintFromRadius(new_config.robot_radius));
  }
}

void ImpreciseCostmap2DROS::setUnpaddedRobotFootprint(const std::vector<geometry_msgs::Point>& points)
{
  unpadded_footprint_ = points;
  padded_footprint_ = points;
  costmap_2d::padFootprint(padded_footprint_, footprint_padding_);

  i_costmap_->setFootprint(padded_footprint_);
  s_costmap_->setFootprint(padded_footprint_);
}

void ImpreciseCostmap2DROS::movementCB(const ros::TimerEvent &event)
{
  // don't allow configuration to happen while this check occurs
  // boost::recursive_mutex::scoped_lock mcl(configuration_mutex_);

  geometry_msgs::PoseStamped new_pose;

  if (!getRobotPose(new_pose))
  {
    ROS_WARN_THROTTLE(1.0, "Could not get robot pose, cancelling reconfiguration");
    robot_stopped_ = false;
  }
  // make sure that the robot is not moving
  else
  {
    old_pose_ = new_pose;

    robot_stopped_ = (tf2::Vector3(old_pose_.pose.position.x, old_pose_.pose.position.y,
                                   old_pose_.pose.position.z).distance(tf2::Vector3(new_pose.pose.position.x,
                                       new_pose.pose.position.y, new_pose.pose.position.z)) < 1e-3) &&
                     (tf2::Quaternion(old_pose_.pose.orientation.x,
                                      old_pose_.pose.orientation.y,
                                      old_pose_.pose.orientation.z,
                                      old_pose_.pose.orientation.w).angle(tf2::Quaternion(new_pose.pose.orientation.x,
                                          new_pose.pose.orientation.y,
                                          new_pose.pose.orientation.z,
                                          new_pose.pose.orientation.w)) < 1e-3);
  }
}

void ImpreciseCostmap2DROS::mapUpdateLoop(double frequency)
{
  ROS_INFO("mapUpdateLoop frequency %.2f",
	   frequency);
  // the user might not want to run the loop every cycle
  if (frequency == 0.0)
    return;

  ros::NodeHandle nh;
  ros::Rate r(frequency);
  while (nh.ok() && !map_update_thread_shutdown_)
  {
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);

    updateMap();

    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    ROS_DEBUG("Map update time: %.9f", t_diff);
    ROS_INFO("publish_cycle %.2f map_initilaized %d\n",
	     publish_cycle.toSec(), s_costmap_->isInitialized());
    if (publish_cycle.toSec() > 0 && s_costmap_->isInitialized())
    {
      unsigned int x0, y0, xn, yn;
      s_costmap_->getBounds(&x0, &xn, &y0, &yn);
      publisher_->updateBounds(x0, xn, y0, yn);

      ros::Time now = ros::Time::now();
      if (last_publish_ + publish_cycle < now)
      {
	ROS_INFO("Publishing maps\n");
        publisher_->publishCostmap();
	low_cmap_publisher_->publishCostmap();
	high_cmap_publisher_->publishCostmap();
	scmap_publisher_->publishCostmap();
        last_publish_ = now;
      }
    }
    r.sleep();
    // make sure to sleep for the remainder of our cycle time
    if (r.cycleTime() > ros::Duration(1 / frequency))
      ROS_WARN("Map update loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", frequency,
               r.cycleTime().toSec());
  }
}

void ImpreciseCostmap2DROS::updateMap()
{
  if (!stop_updates_)
  {
    // get global pose
    geometry_msgs::PoseStamped pose;
    if (getRobotPose (pose))
    {
      double x = pose.pose.position.x,
             y = pose.pose.position.y,
             yaw = tf2::getYaw(pose.pose.orientation);

      i_costmap_->updateMap(x, y, yaw);
      s_costmap_->updateMap(x, y, yaw);

      geometry_msgs::PolygonStamped footprint;
      footprint.header.frame_id = global_frame_;
      footprint.header.stamp = ros::Time::now();
      costmap_2d::transformFootprint(x, y, yaw, padded_footprint_, footprint);
      footprint_pub_.publish(footprint);

      initialized_ = true;
    }
  }
}

void ImpreciseCostmap2DROS::start()
{
  
  std::vector < boost::shared_ptr<costmap_2d::Layer> > *plugins = s_costmap_->getPlugins();
  
  // check if we're stopped or just paused
  if (stopped_)
  {
    // if we're stopped we need to re-subscribe to topics
    
    for (vector<boost::shared_ptr<costmap_2d::Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
        ++plugin)
    {
      (*plugin)->activate();
    }
    
    stopped_ = false;
  }
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
  ros::Rate r(100.0);
  while (ros::ok() && !initialized_)
    r.sleep();
}

void ImpreciseCostmap2DROS::stop()
{
  stop_updates_ = true;
  
  std::vector < boost::shared_ptr<costmap_2d::Layer> > *plugins = s_costmap_->getPlugins();
  
  // unsubscribe from topics
  
  for (vector<boost::shared_ptr<costmap_2d::Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
      ++plugin)
  {
    (*plugin)->deactivate();
  }
  
  initialized_ = false;
  stopped_ = true;
}

void ImpreciseCostmap2DROS::pause()
{
  stop_updates_ = true;
  initialized_ = false;
}

void ImpreciseCostmap2DROS::resume()
{
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
  ros::Rate r(100.0);
  while (!initialized_)
    r.sleep();
}


void ImpreciseCostmap2DROS::resetLayers()
{
  costmap_2d::Costmap2D* top = s_costmap_->getCostmap();
  top->resetMap(0, 0, top->getSizeInCellsX(), top->getSizeInCellsY());
  
  std::vector < boost::shared_ptr<costmap_2d::Layer> > *plugins = s_costmap_->getPlugins();
  for (vector<boost::shared_ptr<costmap_2d::Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
      ++plugin)
  {
    (*plugin)->reset();
  }
  
}

bool ImpreciseCostmap2DROS::getRobotPose(geometry_msgs::PoseStamped& global_pose) const
{
  tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
  geometry_msgs::PoseStamped robot_pose;
  tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
  robot_pose.header.frame_id = robot_base_frame_;
  robot_pose.header.stamp = ros::Time();
  ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

  // get the global pose of the robot
  try
  {
    tf_.transform(robot_pose, global_pose, global_frame_);
  }
  catch (tf2::LookupException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf2::ConnectivityException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf2::ExtrapolationException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  // check global_pose timeout
  if (current_time.toSec() - global_pose.header.stamp.toSec() > transform_tolerance_)
  {
    ROS_WARN_THROTTLE(1.0,
                      "ImpreciseCostmap2DROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
                      current_time.toSec(), global_pose.header.stamp.toSec(), transform_tolerance_);
    return false;
  }

  return true;
}

void ImpreciseCostmap2DROS::getOrientedFootprint(std::vector<geometry_msgs::Point>& oriented_footprint) const
{
  geometry_msgs::PoseStamped global_pose;
  if (!getRobotPose(global_pose))
    return;

  double yaw = tf2::getYaw(global_pose.pose.orientation);
  costmap_2d::transformFootprint(global_pose.pose.position.x, global_pose.pose.position.y, yaw,
                     padded_footprint_, oriented_footprint);
}

  double pdf(const Eigen::Vector2d& x,  Eigen::Vector2d &mean,
  Eigen::Matrix2d &sigma ) 
{
  double sqrt2pi = std::sqrt(2 * M_PI);
  double quadform  = (x - mean).transpose() * sigma.inverse() * (x - mean);
  double norm = std::pow(sqrt2pi, - 2) *
                std::pow(sigma.determinant(), - 0.5);

  return norm * exp(-0.5 * quadform);
}

  
void ImpreciseCostmap2DROS::incomingInput(const geometry_msgs::Point& input)
{
  printf("input\n"); 
  ROS_INFO("Got input pose %.2f %.2f %.2f\n", input.x,
	  input.y, input.z);
  geometry_msgs::PoseStamped global_pose;
  if (!getRobotPose(global_pose))
    return;

  double yaw = tf2::getYaw(global_pose.pose.orientation);

  
  
  //Eigen::Vector2d mean;
  //Eigen::Matrix2d covar;
  //  mean << input.x,input.y; // Set the mean
  //mean << 0, 0;
  // Create a covariance matrix
  // Much wider than it is tall
  // and rotated clockwise by a bit
  //double var = 3.0 / i_costmap_->getLowCostmap()->getResolution();
  //  ROS_INFO("var %.2f", var);
  //covar = genCovar(var,var,0);

 // Create a bivariate gaussian distribution of doubles.
 // with our chosen mean and covariance
  //  const int dim = 2;
  //Eigen::EigenMultivariateNormal<double> normX_solver(mean,covar);
  //int mat_n = static_cast<unsigned int>(ceil(10.0 / i_costmap_->getLowCostmap()->getResolution()));
  //ROS_INFO("mat_n costmap res %.2f (%d)", i_costmap_->getLowCostmap()->getResolution(), mat_n);
  //unsigned int nn = mat_n * mat_n;
  //Eigen::Matrix2d MVNMat =  normX_solver.samples(nn).transpose();
  

  // Let's use a polygon (ellipse)
  std::vector<geometry_msgs::Point> area;
  double off_alpha = atan2(input.y - global_pose.pose.position.y,
			   input.x - global_pose.pose.position.x);
  double r1 = 2;
  double r2 = 1.5;
  for(int i=0; i < 24; i++)
    {
      double alpha = i*M_PI*2/24.;
      geometry_msgs::Point pt;
      // point in ellipse, translated to input
      double px = r1 * cos(alpha);
      double py = r2 * sin(alpha);
      /// we want the ellipse to be facing the robot,
      /// with the longest radius aligned with the y-axis
      double beta = off_alpha + M_PI/2.;
      pt.x = px*cos(beta) - py*sin(beta);
      pt.y = px*sin(beta) + py*cos(beta);

      pt.x += input.x;
      pt.y += input.y;
      area.push_back(pt);
    }
  i_costmap_->getLowCostmap()->setConvexPolygonCost(area, 50);
  i_costmap_->getHighCostmap()->setConvexPolygonCost(area, 85);

 #if 0
  std::vector<geometry_msgs::Point> area;
  for(std::vector<geometry_msgs::Point>::iterator it = unpadded_footprint_.begin(); it != unpadded_footprint_.end(); it++)
    {
      ROS_INFO("Footprint %.2f %.2f",it->x, it->y);
      geometry_msgs::PointStamped ps;
      ps.point = *it;
      ps.point.x += input.x;
      ps.point.y += input.y;
      ps.header.seq = 1;
      ps.header.frame_id = robot_base_frame_;
      ps.header.stamp = ros::Time();

      
      geometry_msgs::PointStamped global_ps;
      global_ps = ps;
      /*
      std::string tf_error;
      if( tf_.canTransform(global_frame_, robot_base_frame_, ros::Time(), ros::Duration(0.1), &tf_error))
	{
	  // get the global pose of the robot
	  try
	    {
	      tf_.transform(ps, global_ps, global_frame_);
	    }
	  catch (tf2::LookupException& ex)
	    {
	      ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
	      return;
	    }
	  catch (tf2::ConnectivityException& ex)
	    {
	      ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
	      return;
	    }
	  catch (tf2::ExtrapolationException& ex)
	    {
	      ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
	      return;
	    }	  
	}
      else
	{
	  ROS_INFO("Can't transform between %s %s\n", global_frame_.c_str(),
		   robot_base_frame_.c_str());	  
	}
      ROS_INFO("Translated footprint in map %.2f %.2f",global_ps.point.x, global_ps.point.y);
      */
      area.push_back(global_ps.point);
      unsigned int mx, my;

      if( !i_costmap_->getLowCostmap()->worldToMap(global_ps.point.x, global_ps.point.y, mx, my))
	{
	  ROS_INFO("Error worldToMap %f %f\n", global_ps.point.x, global_ps.point.y);
	    ROS_INFO("LowCostmap o %.2f %.2f res %.2f map size %d x %d (%.2f x %.2f)",
		     i_costmap_->getLowCostmap()->getOriginX(),
		     i_costmap_->getLowCostmap()->getOriginY(),
		     i_costmap_->getLowCostmap()->getResolution(),
		     i_costmap_->getLowCostmap()->getSizeInCellsX(),
		     i_costmap_->getLowCostmap()->getSizeInCellsY(),
		     i_costmap_->getLowCostmap()->getSizeInMetersX(),
		     i_costmap_->getLowCostmap()->getSizeInMetersY());

	}
      else
	{
	  ROS_INFO("worldToMap succeed %f %f %d %d\n", global_ps.point.x, global_ps.point.y, mx, my);
	}
    }
#endif

  /*
   unsigned int mx, my;

  if( !i_costmap_->getLowCostmap()->worldToMap(input.x, input.y, mx, my))
    {
      ROS_INFO("Error worldToMap %f %f\n", input.x, input.y);
      return;
    }
  else
    {
      ROS_INFO("worldToMap is %d %d\n", mx, my);
    }
  ROS_INFO("Looping xx from %d", mat_n);
  ROS_INFO("Looping from %d", -1*mat_n/2);
  for( int i = -mat_n/2; i < mat_n/2; i++)
    {
      ROS_INFO("i %d", i);
      int mix = mx + i;
      ROS_INFO("mix is %d [SIZEX] %d\n", mix, i_costmap_->getLowCostmap()->getSizeInCellsX());
      if( ! (mix >= 0 &&
	     mix < i_costmap_->getLowCostmap()->getSizeInCellsX() ))
	continue;
      for(int j = -mat_n/2; j<mat_n/2; j++)
	{
	  int miy = my + i;
	  ROS_INFO("miy is %d [SIZEY] %d\n", mix, i_costmap_->getLowCostmap()->getSizeInCellsY());
	  if( ! (miy >= 0 &&
		 miy < i_costmap_->getLowCostmap()->getSizeInCellsY() ))
	    continue;
	  Eigen::Vector2d pt;
	  pt << i, j;
	  double pp = pdf(pt, mean , covar);
	  ROS_INFO("pdf for %d %d is %.2f\n", mix, miy, pp);
	  if( pp > 0)
	    ROS_INFO("POSITIVE pdf for %d %d is %.2f\n", mix, miy, pp);
	  i_costmap_->getLowCostmap()->setCost((unsigned int) mix,
					       (unsigned int) miy,
					       (unsigned char)ceil(100*pp));
	}
    }
  */
}

  
}  // namespace imprecise_costmap_2d
