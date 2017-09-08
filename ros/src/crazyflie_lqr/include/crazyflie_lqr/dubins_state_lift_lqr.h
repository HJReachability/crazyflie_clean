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
 * Authors: David Fridovich-Keil    ( dfk@eecs.berkeley.edu )
 *          Jaime Fernandez Fisac   ( jfisac@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// LQR hover controller for the Crazyflie. Assumes that the state space is
// given by the DubinsStateStamped message type, which is a 7D model but the
// reference is only a 6D PositionStateStamped message type (appends zero yaw).
//
///////////////////////////////////////////////////////////////////////////////

#ifndef CRAZYFLIE_LQR_DUBINS_STATE_LIFT_LQR_H
#define CRAZYFLIE_LQR_DUBINS_STATE_LIFT_LQR_H

#include <crazyflie_lqr/linear_feedback_controller.h>
#include <crazyflie_utils/types.h>
#include <crazyflie_utils/angles.h>
#include <crazyflie_msgs/DubinsStateStamped.h>
#include <crazyflie_msgs/PositionStateStamped.h>
#include <crazyflie_msgs/ControlStamped.h>

#include <ros/ros.h>
#include <math.h>
#include <fstream>

class DubinsStateLiftLqr : public LinearFeedbackController {
public:
  ~DubinsStateLiftLqr() {}
  explicit DubinsStateLiftLqr()
    : LinearFeedbackController() {}

  bool Initialize(const ros::NodeHandle& n);

private:
  // Register callbacks.
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Process an incoming reference point.
  void ReferenceCallback(const crazyflie_msgs::PositionStateStamped::ConstPtr& msg);

  // Process an incoming state measurement.
  void StateCallback(const crazyflie_msgs::DubinsStateStamped::ConstPtr& msg);

  // Process a signal from the in flight topic.
  inline void InFlightCallback(const std_msgs::Empty::ConstPtr& msg) {
    in_flight_ = true;
  }
}; //\class DubinsStateLiftLqr

#endif
