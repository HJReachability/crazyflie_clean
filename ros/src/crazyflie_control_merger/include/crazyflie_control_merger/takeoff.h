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
// Class to provide takeoff service.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef CRAZYFLIE_CONTROL_MERGER_TAKEOFF_H
#define CRAZYFLIE_CONTROL_MERGER_TAKEOFF_H

#include <crazyflie_utils/types.h>
#include <crazyflie_utils/angles.h>
#include <crazyflie_msgs/ControlStamped.h>
#include <crazyflie_msgs/Control.h>
#include <crazyflie_msgs/NoYawControlStamped.h>
#include <crazyflie_msgs/NoYawControl.h>
#include <crazyflie_msgs/PositionStateStamped.h>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <math.h>
#include <fstream>

namespace crazyflie_control_merger {

class Takeoff {
public:
  ~Takeoff() {}
  explicit Takeoff()
    : in_flight_(false),
      initialized_(false) {}

  // Initialize this class.
  bool Initialize(const ros::NodeHandle& n);

private:
  // Load parameters and register callbacks.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Takeoff service. Set in_flight_ flag to true.
  bool TakeoffService(std_srvs::Empty::Request& req,
                      std_srvs::Empty::Response& res);

  // Publishers, subscribers, and topics.
  ros::Publisher control_pub_;
  ros::Publisher in_flight_pub_;
  ros::Publisher reference_pub_;

  std::string control_topic_;
  std::string in_flight_topic_;
  std::string reference_topic_;

  // Takeoff service.
  ros::ServiceServer takeoff_srv_;
  bool in_flight_;

  // Naming and initialization.
  bool initialized_;
  std::string name_;
}; //\class Takeoff

} //\crazyflie_control_merger

#endif
