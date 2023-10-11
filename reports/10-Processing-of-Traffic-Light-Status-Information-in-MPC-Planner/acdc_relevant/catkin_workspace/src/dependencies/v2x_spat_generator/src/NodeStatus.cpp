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

//NodeStatus Callback function 
void spat_generator::NodeStatusCallback(const std_msgs::Bool::ConstPtr& msg)
{
    ROS_INFO_STREAM("NodeStatus Message received");
    
    NodeStatus = msg->data;

	//Initialize traffic light
    start_time_1 = ros::WallTime::now();
	start_time_2 = ros::WallTime::now();

	//control traffic light starting state with rosparam config/sg_1_red & config/sg_2_red 
	if(sg_1_red && sg_2_red)
	{
		current_states[0] = 3;			// (current state of main road traffic light at intersection)   3 --> start with red
		current_states[1] = 6;			// (current state of pedestrian signals at intersection)        6 --> start with green
		current_states[2] = 3;			// (current state of added road traffic light)                  3 --> start with red
		time_to_change_1 = start_time_1 + ros::WallDuration(ttg_1);
		time_to_change_2 = start_time_2 + ros::WallDuration(ttg_2);
	}
	else if(!sg_1_red && sg_2_red)
	{
		current_states[0] = 6;			
		current_states[1] = 3;			
		current_states[2] = 3;	
		time_to_change_1 = start_time_1 + ros::WallDuration(ttr_1);
		time_to_change_2 = start_time_2 + ros::WallDuration(ttg_2);
	}
	else if(sg_1_red && !sg_2_red)
	{
		current_states[0] = 3;			
		current_states[1] = 6;			
		current_states[2] = 6;	
		time_to_change_1 = start_time_1 + ros::WallDuration(ttg_1);
		time_to_change_2 = start_time_2 + ros::WallDuration(ttr_2);
	}
	else if(!sg_1_red && !sg_2_red)
	{
		current_states[0] = 6;			
		current_states[1] = 3;			
		current_states[2] = 6;	
		time_to_change_1 = start_time_1 + ros::WallDuration(ttr_1);
		time_to_change_2 = start_time_2 + ros::WallDuration(ttr_2);
	}

    //shutdown NodeStatus Subscriber immediately after receiving  message
    sub.shutdown();
}

// Fill Spat Function
// ==================================================================
definitions::v2x_SPAT spat_generator::fillSpat()
{
	// Init Spat Message and submessages
	definitions::v2x_SPAT spatOut;
	definitions::v2x_SPAT_IntersectionState intersection;
	definitions::v2x_SPAT_MovementState mvmtStates[nSignalGroups];
	definitions::ASN_bitstring asnBitstring;

	// Time calculations -----------------------------------------
	int soy = ros::WallTime::now().toSec() - epoch_01_2022_;    // second of year
	int moy = soy/60;											// minute of year
	int moh = moy%60;											// minute of hour
	int sec = (soy)%60;											// second of minute    

	//start the timer if NodeStatus == true
    if (NodeStatus)
    {
		//control traffic light timing with rosparam config/ttr_1, config/ttr_2, config/ttg_1, config/ttg_2
        time_to_red_1 = ros::WallDuration(ttr_1);
        time_to_red_2 = ros::WallDuration(ttr_2);
        time_to_green_1 = ros::WallDuration(ttg_1);
        time_to_green_2 = ros::WallDuration(ttg_2);

		// if condition true, signal state changes
        if ((ros::WallTime::now() - time_to_change_1).toSec() > 0){ 
            start_time_1 = ros::WallTime::now();
            if (current_states[0] == 3)
			{
                std::swap(current_states[0], current_states[1]);
                time_to_change_1 = start_time_1 + time_to_red_1;
            }
            else if (current_states[0] == 6)
			{
                std::swap(current_states[0], current_states[1]);
                time_to_change_1 = start_time_1 + time_to_green_1;
            }
        }

		// if condition true, signal state changes
        if ((ros::WallTime::now() - time_to_change_2).toSec() > 0)
		{ 
            start_time_2 = ros::WallTime::now();
            if (current_states[2] == 3)
			{
                current_states[2] = 6;
                time_to_change_2 = start_time_2 + time_to_red_2;
            }
            else if (current_states[2] == 6)
			{
                current_states[2] = 3;
                time_to_change_2 = start_time_2 + time_to_green_2;
            }
        }
    }
	//freeze the timer if NodeStatus == false
    else
    {
        time_to_change_1 = ros::WallTime::now() + ros::WallDuration(99); 
        time_to_change_2 = ros::WallTime::now() + ros::WallDuration(99);
        current_states[0] = 3;
        current_states[1] = 6;
        current_states[2] = 3;
    }
	
	int change_soy_1 = time_to_change_1.toSec() - epoch_01_2022_;	// second of year
	int change_moy_1 = change_soy_1/60;								// minute of year
	int change_moh_1 = change_moy_1%60;								// minute of hour
	int change_sec_1 = (change_soy_1)%60;							// second of minute

	int change_soy_2 = time_to_change_2.toSec() - epoch_01_2022_;	// second of year
	int change_moy_2 = change_soy_2/60;								// minute of year
	int change_moh_2 = change_moy_2%60;								// minute of hour
	int change_sec_2 = (change_soy_2)%60;

	// Fill message -------------------------------------------------

	// ASN Bitstring
	asnBitstring.buf.push_back(1);
	asnBitstring.bits_unused = 0;

	// Event
	for (int i = 0; i < nSignalGroups; i++) {
		if(i < 2)
		{
			mvmtStates[i].signalGroup = i+1;
			mvmtStates[i].movementName = "";
			mvmtStates[i].movementName_present = false;
			
			definitions::v2x_SPAT_MovementEvent mvmtEvent;
			mvmtEvent.eventState = current_states[i];
			mvmtEvent.timing_startTime = 0;
			mvmtEvent.timing_startTime_present = false;
			mvmtEvent.timing_minEndTime = (change_moh_1*60+change_sec_1)*10-1;
			mvmtEvent.timing_likelyTime = (change_moh_1*60+change_sec_1)*10;
			mvmtEvent.timing_likelyTime_present = true;
			mvmtEvent.timing_maxEndTime = (change_moh_1*60+change_sec_1)*10+1;
			mvmtEvent.timing_maxEndTime_present = true;
			mvmtEvent.timing_present = true;

			mvmtStates[i].state_time_speed.push_back(mvmtEvent);
			intersection.states.push_back(mvmtStates[i]);
		}
		else
		{
			mvmtStates[i].signalGroup = i+1;
			mvmtStates[i].movementName = "";
			mvmtStates[i].movementName_present = false;
			
			definitions::v2x_SPAT_MovementEvent mvmtEvent;
			mvmtEvent.eventState = current_states[i];
			mvmtEvent.timing_startTime = 0;
			mvmtEvent.timing_startTime_present = false;
			mvmtEvent.timing_minEndTime = (change_moh_2*60+change_sec_2)*10-1;
			mvmtEvent.timing_likelyTime = (change_moh_2*60+change_sec_2)*10;
			mvmtEvent.timing_likelyTime_present = true;
			mvmtEvent.timing_maxEndTime = (change_moh_2*60+change_sec_2)*10+1;
			mvmtEvent.timing_maxEndTime_present = true;
			mvmtEvent.timing_present = true;

			mvmtStates[i].state_time_speed.push_back(mvmtEvent);
			intersection.states.push_back(mvmtStates[i]);
		}
	}
	

	// Intersection
	if (intersection_name != ""){
		intersection.name = intersection_name;
		intersection.name_present = true;
	}
	else{
		intersection.name_present = false;
	}
	if (id_region != -1){
		intersection.id_region = id_region;
		intersection.id_region_present = true;
	}
	else{
		intersection.id_region_present = false;
	}
	intersection.id_id = id_id;
	intersection.revision = 1;
	intersection.moy = moy;
	intersection.moy_present = true;
	intersection.timeStamp = sec;
	intersection.timeStamp_present = true;
	intersection.status = asnBitstring;
	intersection.enabledLanes_present = false;
	intersection.maneuverAssistList_present = false;
	intersection.priority_present = false;
	intersection.preempt_present = false;
	intersection.regional_present = false;

	// Main Spat
	spatOut.header_protocolVersion = 2;
	spatOut.header_messageID = 4; // SPAT
	spatOut.header_stationID = stationID;
	spatOut.spatData_msgID = 0;
    spatOut.spatData_msgSubID = 0;
    spatOut.spatData_msgSubID_present = false;
    if (spat_name != ""){
		spatOut.spatData_name = spat_name;
    	spatOut.spatData_name_present = true;
	}
	else{
		spatOut.spatData_name_present = false;
	}
    spatOut.spatData_regional_present = false;

	spatOut.spatData_intersections.push_back(intersection);

	//ROS_INFO("FillSpat: [%d]",NodeStatus);

	return spatOut;
}

}