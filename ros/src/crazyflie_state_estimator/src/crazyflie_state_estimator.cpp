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
// State estimator node. Sets a recurring timer and every time it rings,
// this node will query tf for the transform between the specified robot
// frame and the fixed frame, merge it with the previous state estimate, and
// publish the new estimate.
//
///////////////////////////////////////////////////////////////////////////////

#include <crazyflie_state_estimator/crazyflie_state_estimator.h>

const size_t CrazyflieStateEstimator::X_DIM = 12;

// Initialize this class by reading parameters and loading callbacks.
bool CrazyflieStateEstimator::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "crazyflie_lqr");

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
bool CrazyflieStateEstimator::LoadParameters(const ros::NodeHandle& n) {
  std::string key;

  // State topic.
  if (!ros::param::search("state_topic", key)) return false;
  if (!ros::param::get(key, state_topic_)) return false;

  // Frames of reference.
  if (!ros::param::search("fixed_frame_id", key)) return false;
  if (!ros::param::get(key, fixed_frame_id_)) return false;

  if (!ros::param::search("robot_frame_id", key)) return false;
  if (!ros::param::get(key, robot_frame_id_)) return false;

  // Time step for reading tf.
  if (!ros::param::search("time_step", key)) return false;
  if (!ros::param::get(key, dt_)) return false;

  return true;
}

// Register callbacks.
bool CrazyflieStateEstimator::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // State publisher.
  state_pub_ = nl.advertise<crazyflie_msgs::StateStamped>(
    state_topic_.c_str(), 10, false);

  // Timer.
  timer_ = nl.createTimer(
    ros::Duration(dt_), &CrazyflieStateEstimator::TimerCallback, this);

  return true;
}

// Whenever timer rings, query tf, update state estimate, and publish.
void CrazyflieStateEstimator::TimerCallback(const ros::TimerEvent& e) {
  const ros::Time right_now = ros::Time::now();

  // Get the current transform from tf.
  geometry_msgs::TransformStamped tf;

  try {
    tf = tf_buffer_.lookupTransform(
      fixed_frame_id_.c_str(), robot_frame_id_.c_str(), right_now);
  } catch(tf2::TransformException &ex) {
    ROS_WARN("%s: %s", name_.c_str(), ex.what());
    ROS_WARN("%s: Could not determine current state.", name_.c_str());
    return;
  }

  // Get roll, pitch, and yaw from quaternion.
  const Quaterniond quat(tf.transform.rotation.w,
                         tf.transform.rotation.x,
                         tf.transform.rotation.y,
                         tf.transform.rotation.z);
  Vector3d euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);
  euler(0) = angles::WrapAngleRadians(euler(0));
  euler(1) = angles::WrapAngleRadians(euler(1));
  euler(2) = angles::WrapAngleRadians(euler(2));

  // Catch first update.
  if (first_update_) {
    x_(0) = tf.transform.translation.x;
    x_(1) = tf.transform.translation.y;
    x_(2) = tf.transform.translation.z;

    x_(3) = 0.0;
    x_(4) = 0.0;
    x_(5) = 0.0;

    x_(6) = euler(0);
    x_(7) = euler(1);
    x_(8) = euler(2);

    x_(9) = 0.0;
    x_(10) = 0.0;
    x_(11) = 0.0;

    first_update_ = false;
  } else {
    // Time difference.
    const double dt = (right_now - last_time_).toSec();

    // TODO! Use a smoothing filter here instead.
    // Update velocities.
    x_(3) = (tf.transform.translation.x - x_(0)) / dt;
    x_(4) = (tf.transform.translation.y - x_(1)) / dt;
    x_(5) = (tf.transform.translation.z - x_(2)) / dt;

    x_(9) = (euler(0) - x_(6)) / dt;
    x_(10) = (euler(1) - x_(7)) / dt;
    x_(11) = (euler(2) - x_(8)) / dt;

    // Update position/orientation.
    x_(0) = tf.transform.translation.x;
    x_(1) = tf.transform.translation.y;
    x_(2) = tf.transform.translation.z;

    x_(6) = euler(0);
    x_(7) = euler(1);
    x_(8) = euler(2);
  }

  // Update the time.
  last_time_ = right_now;

  // Publish.
  crazyflie_msgs::StateStamped msg;

  msg.header.frame_id = fixed_frame_id_;
  msg.header.stamp = right_now;

  msg.state.x = x_(0);
  msg.state.y = x_(1);
  msg.state.z = x_(2);
  msg.state.x_dot = x_(3);
  msg.state.y_dot = x_(4);
  msg.state.z_dot = x_(5);
  msg.state.roll = x_(6);
  msg.state.pitch = x_(7);
  msg.state.yaw = x_(8);
  msg.state.roll_dot = x_(9);
  msg.state.pitch_dot = x_(10);
  msg.state.yaw_dot = x_(11);

  state_pub_.publish(msg);
}
