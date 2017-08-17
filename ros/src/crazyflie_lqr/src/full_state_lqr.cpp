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
// LQR controller for the Crazyflie. Uses an LQR control matrix for hovering
// at each specified reference point.
//
///////////////////////////////////////////////////////////////////////////////

#include <crazyflie_lqr/crazyflie_lqr.h>

const size_t CrazyflieLQR::U_DIM = 7;
const size_t CrazyflieLQR::X_DIM = 12;

// Initialize this class by reading parameters and loading callbacks.
bool CrazyflieLQR::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "crazyflie_lqr");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // Set up file io to read K, x_ref, and u_ref from disk.
  std::ifstream K_file(K_filename_);
  std::ifstream x_ref_file(x_ref_filename_);
  std::ifstream u_ref_file(u_ref_filename_);

  // Read K.
  if (K_file.is_open()) {
    for (size_t ii = 0; ii < U_DIM && K_file.good(); ii++)
      for (size_t jj = 0; jj < X_DIM && K_file.good(); jj++)
        K_file >> K_(ii, jj);
  } else {
    ROS_ERROR("%s: Could not find %s.", name_.c_str(), K_filename_.c_str());
    return false;
  }

  // Read x_ref.
  if (x_ref_file.is_open()) {
    for (size_t ii = 0; ii < X_DIM && x_ref_file.good(); ii++)
      x_ref_file >> x_ref_(ii);
  } else {
    ROS_ERROR("%s: Could not find %s.", name_.c_str(), x_ref_filename_.c_str());
    return false;
  }

  // Read x_ref.
  if (u_ref_file.is_open()) {
    for (size_t ii = 0; ii < U_DIM && u_ref_file.good(); ii++)
      u_ref_file >> u_ref_(ii);
  } else {
    ROS_ERROR("%s: Could not find %s.", name_.c_str(), u_ref_filename_.c_str());
    return false;
  }

  initialized_ = true;
  return true;
}

// Load parameters.
bool CrazyflieLQR::LoadParameters(const ros::NodeHandle& n) {
  std::string key;

  // Text files with K, x_ref, u_ref.
  if (!ros::param::search("crazyflie_lqr/K_file", key)) return false;
  if (!ros::param::get(key, K_filename_)) return false;

  if (!ros::param::search("crazyflie_lqr/u_ref_file", key)) return false;
  if (!ros::param::get(key, u_ref_filename_)) return false;

  if (!ros::param::search("crazyflie_lqr/x_ref_file", key)) return false;
  if (!ros::param::get(key, x_ref_filename_)) return false;

  // Topics.
  if (!ros::param::search("crazyflie_lqr/state_topic", key)) return false;
  if (!ros::param::get(key, state_topic_)) return false;

  if (!ros::param::search("crazyflie_lqr/reference_topic", key)) return false;
  if (!ros::param::get(key, reference_topic_)) return false;

  if (!ros::param::search("crazyflie_lqr/control_topic", key)) return false;
  if (!ros::param::get(key, control_topic_)) return false;

  return true;
}

// Register callbacks.
bool CrazyflieLQR::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Subscribers.
  state_sub_ = nl.subscribe(
    state_topic_.c_str(), 10, &CrazyflieLQR::StateCallback, this);
  reference_sub_ = nl.subscribe(
    reference_topic_.c_str(), 10, &CrazyflieLQR::ReferenceCallback, this);

  // Control publisher.
  control_pub_ = nl.advertise<crazyflie_msgs::ControlStamped>(
    control_topic_.c_str(), 10, false);

  return true;
}

// Process an incoming reference point change.
void CrazyflieLQR::ReferenceCallback(
  const crazyflie_msgs::FullStateStamped::ConstPtr& msg) {
  x_ref_(0) = msg->state.x;
  x_ref_(1) = msg->state.y;
  x_ref_(2) = msg->state.z;
  x_ref_(3) = msg->state.x_dot;
  x_ref_(4) = msg->state.y_dot;
  x_ref_(5) = msg->state.z_dot;
  x_ref_(6) = msg->state.roll;
  x_ref_(7) = msg->state.pitch;
  x_ref_(8) = msg->state.yaw;
  x_ref_(9) = msg->state.roll_dot;
  x_ref_(10) = msg->state.pitch_dot;
  x_ref_(11) = msg->state.yaw_dot;

  ROS_INFO("%s: Set new reference point.", name_.c_str());
}

// Process an incoming state measurement.
void CrazyflieLQR::StateCallback(
  const crazyflie_msgs::FullStateStamped::ConstPtr& msg) {
  // Read the message into the state and compute relative state.
  VectorXd x(12);
  x(0) = msg->state.x;
  x(1) = msg->state.y;
  x(2) = msg->state.z;
  x(3) = msg->state.x_dot;
  x(4) = msg->state.y_dot;
  x(5) = msg->state.z_dot;
  x(6) = msg->state.roll;
  x(7) = msg->state.pitch;
  x(8) = msg->state.yaw;
  x(9) = msg->state.roll_dot;
  x(10) = msg->state.pitch_dot;
  x(11) = msg->state.yaw_dot;

  VectorXd x_rel = x - x_ref_;

  ROS_INFO("%s: StateCallback was called.", name_.c_str());
  std::cout << "Relative state: " << x_rel.transpose() << std::endl;


  // Rotate x and y coordinates.
  const double cos_y = std::cos(x(8));
  const double sin_y = std::sin(x(8));

  const double rot_x = cos_y * x_rel(0) + sin_y * x_rel(1);
  const double rot_y = -sin_y * x_rel(0) + cos_y * x_rel(1);
  const double rot_x_dot = cos_y * x_rel(3) + sin_y * x_rel(4);
  const double rot_y_dot = -sin_y * x_rel(3) + cos_y * x_rel(4);

  x_rel(0) = rot_x;
  x_rel(1) = rot_y;
  x_rel(2) = rot_x_dot;
  x_rel(3) = rot_y_dot;

  // Wrap angles.
  x_rel(6) = angles::WrapAngleRadians(x_rel(6));
  x_rel(7) = angles::WrapAngleRadians(x_rel(7));
  x_rel(8) = angles::WrapAngleRadians(x_rel(8));

  // Compute optimal control.
  VectorXd u = K_ * x_rel + u_ref_;
  u(0) = angles::WrapAngleRadians(u(0));
  u(1) = angles::WrapAngleRadians(u(1));
  u(2) = angles::WrapAngleRadians(u(2));

  std::cout << "Control: " << u.transpose() << std::endl;

  // Publish.
  crazyflie_msgs::ControlStamped control_msg;
  control_msg.control.roll = u(0);
  control_msg.control.pitch = u(1);
  control_msg.control.yaw_dot = u(2);
  control_msg.control.thrust = u(3);

  control_pub_.publish(control_msg);
}
