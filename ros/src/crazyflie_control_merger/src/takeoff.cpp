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
// Class to handle takeoff service.
//
///////////////////////////////////////////////////////////////////////////////

#include <crazyflie_control_merger/takeoff.h>

namespace crazyflie_control_merger {

// Initialize this node.
bool Takeoff::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "takeoff");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  initialized_ = true;
  return true;
}

// Load parameters.
bool Takeoff::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Topics.
  if (!nl.getParam("topics/control", control_topic_)) return false;
  if (!nl.getParam("topics/in_flight", in_flight_topic_)) return false;
  if (!nl.getParam("topics/reference", reference_topic_)) return false;

  return true;
}

// Register callbacks.
bool Takeoff::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Publishers.
  control_pub_ = nl.advertise<crazyflie_msgs::ControlStamped>(
    control_topic_.c_str(), 10, false);

  reference_pub_ = nl.advertise<crazyflie_msgs::PositionStateStamped>(
    reference_topic_.c_str(), 10, false);

  in_flight_pub_ = nl.advertise<std_msgs::Empty>(
    in_flight_topic_.c_str(), 10, false);

  // Services.
  takeoff_srv_ = nl.advertiseService(
    "/takeoff", &Takeoff::TakeoffService, this);

  return true;
}

// Takeoff service. Set in_flight_ flag to true.
bool Takeoff::
TakeoffService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  if (in_flight_) {
    ROS_WARN("%s: Tried to takeoff while in flight.", name_.c_str());
    return false;
  }

  ROS_INFO("%s: Takeoff requested.", name_.c_str());

  // Lift off, and after a short wait return.
#if 0
  const ros::Time right_now = ros::Time::now();
  while ((ros::Time::now() - right_now).toSec() < 2.0) {
    crazyflie_msgs::ControlStamped msg;
    msg.header.stamp = ros::Time::now();

    msg.control.roll = 0.0;
    msg.control.pitch = 0.0;
    msg.control.yaw_dot = 0.0;

    // Offset gravity, plus a little extra to lift off.
    msg.control.thrust = crazyflie_utils::constants::G + 0.2;
    control_pub_.publish(msg);

    // Sleep a little, then rerun the loop.
    ros::Duration(0.01).sleep();
  }
#endif

  // Send reference to LQR.
  // HACK! Read this hover point in from the parameter server.
  crazyflie_msgs::PositionStateStamped reference;
  reference.header.stamp = ros::Time::now();
  reference.state.x = 0.0;
  reference.state.y = 0.0;
  reference.state.z = 0.5;
  reference.state.x_dot = 0.0;
  reference.state.y_dot = 0.0;
  reference.state.z_dot = 0.0;

  reference_pub_.publish(reference);

  in_flight_ = true;

  // Give LQR time to get there.
  ros::Duration(10.0).sleep();

  // Send the in_flight signal to all other nodes!
  in_flight_pub_.publish(std_msgs::Empty());

  // Return true.
  return true;
}

} //\namespace crazyflie_control_merger
