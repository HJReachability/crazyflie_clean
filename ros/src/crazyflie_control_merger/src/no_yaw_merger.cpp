/*
 * Copyright (c) 2017, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Class to merge control messages from two different controllers into
// a single ControlStamped message.
//
///////////////////////////////////////////////////////////////////////////////

#include <crazyflie_control_merger/no_yaw_merger.h>

namespace crazyflie_control_merger {

// Initialize this node.
bool NoYawMerger::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "no_yaw_merger");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // Delay a little while just to make sure other nodes are started up.
  //  ros::Duration(0.5).sleep();

  initialized_ = true;
  return true;
}

// Load parameters.
bool NoYawMerger::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Time step.
  if (!nl.getParam("time_step", dt_)) return false;

  // Topics.
  if (!nl.getParam("topics/control", control_topic_)) return false;
  if (!nl.getParam("topics/prioritized_control", no_yaw_control_topic_))
    return false;
  if (!nl.getParam("topics/merged", merged_topic_)) return false;

  // Mode.
  std::string mode;
  if (!nl.getParam("mode", mode)) return false;

  if (mode == "MERGE") {
    mode_ = MERGE;
    ROS_INFO("%s: Entering MERGE mode.", name_.c_str());
  } else if (mode == "LQR") {
    mode_ = LQR;
    ROS_INFO("%s: Entering LQR mode.", name_.c_str());
  } else if (mode == "OPTIMAL") {
    mode_ = OPTIMAL;
    ROS_INFO("%s: Entering OPTIMAL mode.", name_.c_str());
  } else {
    ROS_ERROR("%s: Invalid mode. Using LQR.", name_.c_str());
    mode_ = LQR;
  }

  return true;
}

// Register callbacks.
bool NoYawMerger::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Subscribers.
  control_sub_ = nl.subscribe(
    control_topic_.c_str(), 1, &NoYawMerger::ControlCallback, this);

  no_yaw_control_sub_ = nl.subscribe(
    no_yaw_control_topic_.c_str(), 1, &NoYawMerger::NoYawControlCallback, this);

  // Publishers.
  merged_pub_ = nl.advertise<crazyflie_msgs::ControlStamped>(
    merged_topic_.c_str(), 1, false);

    // Services.
  land_srv_ = nl.advertiseService("/land", &NoYawMerger::LandService, this);

  // Timer.
  timer_ = nl.createTimer(ros::Duration(dt_), &NoYawMerger::TimerCallback, this);

  return true;
}

// Process an incoming reference point.
void NoYawMerger::ControlCallback(
  const crazyflie_msgs::ControlStamped::ConstPtr& msg) {
  control_ = msg->control;
  control_been_updated_ = true;
}

// Process an incoming state measurement.
void NoYawMerger::NoYawControlCallback(
  const crazyflie_msgs::NoYawControlStamped::ConstPtr& msg) {
  no_yaw_control_ = msg->control;
  no_yaw_control_been_updated_ = true;
}

// Timer callback.
void NoYawMerger::TimerCallback(const ros::TimerEvent& e) {
  if (landed_)
    return;

  crazyflie_msgs::ControlStamped msg;
  msg.header.stamp = ros::Time::now();

  if (!control_been_updated_) {
    // Takeoff node may be operating.. just return.
    return;
  } else if (!no_yaw_control_been_updated_) {
    msg.control = control_;
  } else {
    // Extract no yaw priority.
    double p = no_yaw_control_.priority;
    if (mode_ == LQR)
      p = 0.0;
    else if (mode_ == OPTIMAL)
      p = 1.0;

    // Set message fields.
    msg.control.roll = (1.0 - p) * control_.roll + p * no_yaw_control_.roll;
    msg.control.pitch = (1.0 - p) * control_.pitch + p * no_yaw_control_.pitch;
    msg.control.yaw_dot = control_.yaw_dot;
    msg.control.thrust = (1.0 - p) * control_.thrust + p * no_yaw_control_.thrust;
  }

  merged_pub_.publish(msg);
}

// Landing service.
bool NoYawMerger::
LandService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  ROS_INFO("%s: Landing requested.", name_.c_str());

  const ros::Time right_now = ros::Time::now();
  while ((ros::Time::now() - right_now).toSec() < 1.0) {
    crazyflie_msgs::ControlStamped msg;
    msg.header.stamp = ros::Time::now();

    msg.control.roll = 0.0;
    msg.control.pitch = 0.0;
    msg.control.yaw_dot = 0.0;

    // Slowly decrement thrust.
    msg.control.thrust = std::max(0.0, crazyflie_utils::constants::G -
                                  5.0 * (ros::Time::now() - right_now).toSec());

    merged_pub_.publish(msg);

    // Sleep a little, then rerun the loop.
    ros::Duration(0.01).sleep();
  }

  landed_ = true;

  // Return true.
  return true;
}

} //\namespace crazyflie_control_merger
