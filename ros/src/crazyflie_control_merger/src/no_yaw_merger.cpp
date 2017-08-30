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

  been_updated_ = false;
  initialized_ = true;
  return true;
}

// Load parameters.
bool NoYawMerger::LoadParameters(const ros::NodeHandle& n) {
  std::string key;

  // Time step.
  if (!ros::param::search("merger/time_step", key)) return false;
  if (!ros::param::get(key, dt_)) return false;

  // Topics.
  if (!ros::param::search("merger/topics/control", key)) return false;
  if (!ros::param::get(key, control_topic_)) return false;

  if (!ros::param::search("merger/topics/prioritized_control", key)) return false;
  if (!ros::param::get(key, no_yaw_control_topic_)) return false;

  if (!ros::param::search("merger/topics/merged", key)) return false;
  if (!ros::param::get(key, merged_topic_)) return false;

  return true;
}

// Register callbacks.
bool NoYawMerger::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Subscribers.
  control_sub_ = nl.subscribe(
    control_topic_.c_str(), 10, &NoYawMerger::ControlCallback, this);

  no_yaw_control_sub_ = nl.subscribe(
    no_yaw_control_topic_.c_str(), 10, &NoYawMerger::NoYawControlCallback, this);

  // Publisher.
  merged_pub_ = nl.advertise<crazyflie_msgs::ControlStamped>(
    merged_topic_.c_str(), 10, false);

  return true;
}

// Process an incoming reference point.
void NoYawMerger::ControlCallback(
  const crazyflie_msgs::ControlStamped::ConstPtr& msg) {
  control_ = msg->control;
  been_updated_ = true;
}

// Process an incoming state measurement.
void NoYawMerger::NoYawControlCallback(
  const crazyflie_msgs::NoYawControlStamped::ConstPtr& msg) {
  no_yaw_control_ = msg->control;
  been_updated_ = true;
}

// Timer callback.
void NoYawMerger::TimerCallback(const ros::TimerEvent& e) {
  if (!been_updated_)
    return;

  // Extract no yaw priority
  const double p = no_yaw_control_.priority;

  // Set message fields.
  crazyflie_msgs::ControlStamped msg;
  msg.header.stamp = ros::Time::now();

  msg.control.roll = p * control_.roll + (1.0 - p) * no_yaw_control_.roll;
  msg.control.pitch = p * control_.pitch + (1.0 - p) * no_yaw_control_.pitch;
  msg.control.yaw_dot = control_.yaw_dot;
  msg.control.thrust = p * control_.thrust + (1.0 - p) * no_yaw_control_.thrust;

  merged_pub_.publish(msg);
}

} //\namespace crazyflie_control_merger
