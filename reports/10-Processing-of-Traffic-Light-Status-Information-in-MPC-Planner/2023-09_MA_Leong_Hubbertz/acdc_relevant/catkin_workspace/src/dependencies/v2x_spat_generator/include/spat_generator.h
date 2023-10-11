// Copyright (c) 2022 Institute for Automotive Engineering of RWTH Aachen University

// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

#include <string>
#include <ros/ros.h>
#include "std_msgs/Bool.h"

#include <definitions/v2x_SPAT.h>
#include <definitions/v2x_SPAT_IntersectionState.h>
#include <definitions/v2x_SPAT_MovementState.h>
#include <definitions/v2x_SPAT_MovementEvent.h>
#include <definitions/ASN_bitstring.h>

namespace spat_node {

class spat_generator{
protected:

  ros::Publisher pub_spat_;
  ros::Subscriber sub; 

  int epoch_01_2022_ = 1640995200;
  bool NodeStatus = false;

  // Params
  bool sg_1_red;
  bool sg_2_red;
  double frequency;
  std::string topic_out;
  double ttg_1;
  double ttr_1;
  double ttg_2;
  double ttr_2;

  // Custom SPATEM
  std::string spat_name;
  int stationID;
  std::string intersection_name;
  int id_region;
  int id_id;
  int nSignalGroups;
  
  // Important variables
  ros::WallTime start_time_1;
  ros::WallTime start_time_2;
  ros::WallTime time_to_change_1;
  ros::WallTime time_to_change_2;
  ros::WallDuration time_to_green_1;
  ros::WallDuration time_to_red_1;
  ros::WallDuration time_to_green_2;
  ros::WallDuration time_to_red_2;
  int current_states[3];

public:
  spat_generator();

  ~spat_generator();

  definitions::v2x_SPAT fillSpat();

  void NodeStatusCallback(const std_msgs::Bool::ConstPtr& msg);

  int init(int argc, char **argv);
};

}