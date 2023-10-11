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

#include <spat_generator.h>

namespace spat_node {

// Constructor
spat_generator::spat_generator()
{

}
// Destructor
spat_generator::~spat_generator()
{

}

// Main function
int spat_generator::init(int argc, char **argv)
{

    // Enable Parameter Support in Nodehandle
    ros::NodeHandle n("~");

	// Get General Parameters
	n.param<double>("config/frequency", frequency, 1.0);
	n.param<bool>("config/sg_1_red", sg_1_red, true);
	n.param<bool>("config/sg_2_red", sg_2_red, true);
	n.param<std::string>("config/topic_out", topic_out, "/SPATEMs");
	n.param<double>("config/time_to_green_1", ttg_1, 35.0); 
	n.param<double>("config/time_to_red_1", ttr_1, 20.0);
	n.param<double>("config/time_to_green_2", ttg_2, 55.0); 
	n.param<double>("config/time_to_red_2", ttr_2, 40.0);  

	ROS_INFO("Publish on topic: %s with %f Hz", topic_out.c_str(), frequency);
	ROS_INFO("T1. Time to green: %f s", ttg_1);
	ROS_INFO("T1. Time to red: %f s", ttr_1);
	ROS_INFO("T2. Time to green: %f s", ttg_2);
	ROS_INFO("T2. Time to red: %f s", ttr_2);

	// Get custom SPATEM
	n.param<std::string>("spat_data/name", spat_name, "");
	n.param<int>("spat_data/stationID", stationID, -1);
	n.param<std::string>("spat_data/intersection/name", intersection_name, "");
	n.param<int>("spat_data/intersection/id_region", id_region, -1);
	n.param<int>("spat_data/intersection/id_id", id_id, -1);
	n.param<int>("spat_data/intersection/nSignalGroups", nSignalGroups, 3);

	// Init loopRate
	ros::Rate loopRate(frequency);

	// Init publisher
	pub_spat_ = n.advertise<definitions::v2x_SPAT>(topic_out, 0);
	sub = n.subscribe("/NodeStatus", 1, &spat_generator::NodeStatusCallback, this);

    // Start main loop
    while(ros::ok())
    {
		//ROS_INFO("MainFuction: [%d]",NodeStatus);
    	// Fill spat message
    	definitions::v2x_SPAT spatMessage = fillSpat();

		// Publish spat message
		pub_spat_.publish(spatMessage);

		ros::spinOnce();

    	// Sleep
    	loopRate.sleep();

    }

    // End process
    return 0;

}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spat_generator_node");

  spat_node::spat_generator node;
  node.init(argc, argv);
}