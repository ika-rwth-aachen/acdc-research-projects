#include <map_generator.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_generator_node");

  map_node::map_generator node;
  node.init(argc, argv);
}

namespace map_node {
// Constructor
map_generator::map_generator()
{

}
// Destructor
map_generator::~map_generator()
{

}

//Creates 12 consecutive lane points based on the first point of each lanes at intersection
std::vector<std::vector<std::vector<float>>> map_generator::generate_lane_points(std::vector<std::vector<float>> xy, float d) {
    std::vector<std::vector<std::vector<float>>> result;

    for (int i = 0; i < xy.size(); i++) {
        std::vector<std::vector<float>> list;

        if ((i < 2 || (i > 5 && i < 8))) 
        {
            for (int j = 0; j < 12; j++) 
            {
                float new_x = xy[i][0] - d * j;
                float new_y = xy[i][1];
                list.push_back({new_x, new_y});
            }
            if (i < 2) 
            {
                std::reverse(list.begin(), list.end());
            }
        } 

        else if (i == 5 || i == 11)
        {
            for (int j = 0; j < 3; j++)
            {
                float new_x = xy[i][0];
                float new_y = xy[i][1] + d*j;
                list.push_back({new_x, new_y});               
            }
            std::reverse(list.begin(), list.end());
        }
        else if (i == 4 || i == 10)
        {
            for (int j = 0; j < 3; j++)
            {
                float new_x = xy[i][0];
                float new_y = xy[i][1] - d*j;
                list.push_back({new_x, new_y});
                
            }
            std::reverse(list.begin(), list.end());
        } 
        
        else 
        {
            for (int j = 0; j < 12; j++) 
            {
                float new_x = xy[i][0] + d * j;
                float new_y = xy[i][1];
                list.push_back({new_x, new_y});
            }
            if (i > 1 && i < 4) 
            {
                std::reverse(list.begin(), list.end());
            }
        }

        result.push_back(list);
    }

    return result;
}

//transform all lane points at intersection from local to global coordinate 
std::vector<std::vector<std::vector<float>>> map_generator::local_to_glob_coordinates(std::vector<std::vector<std::vector<float>>> list_xy) {
    std::vector<std::vector<std::vector<float>>> list_glob_f;
    std::vector<std::vector<float>> list_glob;

    for (int i = 0; i < list_xy.size(); i++) {
        for (int j = 0; j < list_xy[i].size(); j++) {
            float new_x = list_xy[i][j][0] * cos(40.0 * PI / 180.0) - list_xy[i][j][1] * sin(40.0 * PI / 180.0);
            float new_y = list_xy[i][j][0] * sin(40.0 * PI / 180.0) + list_xy[i][j][1] * cos(40.0 * PI / 180.0);

            list_glob.push_back({new_x, new_y});
        }

        list_glob_f.push_back(list_glob);
        list_glob.clear();
    }

    return list_glob_f;
}

std::vector<std::vector<std::vector<float>>> map_generator::get_coordinates_list(std::vector<std::vector<float>> x_y_add, float d, std::vector<float> angle) 
{
/*

Creates 12 consecutive lane points based on the coordinate of the starting point, constant distance between each points and the incline angle

Arguments:
x_y_add -- 2-Dimentional vector that contains the initial x, y coordinates of every new ingress lanes
d       -- constant distance between each lane points
angle   -- 1-Dimentional vector that contains the incline angle of each ingress lane

Output:
new_coordinates_final -- a 3-Dimentional vector that stores the x, y coordinates of the 12 lane points of the new ingress lanes

*/

    std::vector<std::vector<std::vector<float>>> new_coordinates_final;
    std::vector<std::vector<float>> new_coordinates;
    
    //outer loop iterates through each row (initial x and y coordinates) of the x_y_add vector
    for (int i = 0; i < x_y_add.size(); i++) {

        //the inner loop iterates 12 times to generate 12 consecutive lane points for each starting point
        for (int j = 0; j < 12; j++) {

            //calculate new x and y coordinates for next lane point, based on the starting point coordinates, value of d and angle
            float new_x = x_y_add[i][0] + d * cos(angle[i] * (PI / 180.0)) * j;
            float new_y = x_y_add[i][1] + d * sin(angle[i] * (PI / 180.0)) * j;

            //appends the newly calculated x and y coordinates to the 'new_coordinates' vector
            new_coordinates.push_back({new_x, new_y});
        }
        //inner loop ends after generating 12 consecutive lane points

        //appends the 'new_coordinates' vector(containing 12 lane points) to the 'new_coordinates_final' vector
        new_coordinates_final.push_back(new_coordinates);
        //clear the 'new_coordinates' vector to be used for next starting point
        new_coordinates.clear();
    }

    return new_coordinates_final;
}

definitions::v2x_MAP map_generator::fillMap(std::vector<std::vector<std::vector<float>>> list_xy)
{
    //Initialize MAP message and sub messages
    definitions::v2x_MAP mapout;
    definitions::v2x_MAP_Intersection intersections;
    definitions::v2x_MAP_Lane adjacent_lanes[list_xy.size()];
    definitions::v2x_MAP_Connection connection;

    //header
    intersections.header.seq = 0;
    intersections.header.frame_id = frame_id;
    intersections.header.stamp.sec = 1656598130;
    intersections.header.stamp.nsec = 187959949;
    
    //adjacent_lanes
    for (int i = 0; i < list_xy.size(); i++){
        adjacent_lanes[i].laneId = i+1;

        if (i < 6 || i > 11 && i < 15)
        {
		    adjacent_lanes[i].directionalUse = 0;
        }
        else
        {
            adjacent_lanes[i].directionalUse = 1;
        }
        adjacent_lanes[i].laneType = 0;

        definitions::v2x_MAP_Connection connection;
        std::vector<int> connectingLane_laneId{9, 10, 7, 8, 12, 11};
            if (i < 4)
            {
                connection.signalGroupId = 1;
                connection.intersectionId = 1234;
                connection.signalGroupId_present = true;
                connection.connectingLane_laneId = connectingLane_laneId[i];
                adjacent_lanes[i].connections.push_back(connection);
            }
            else if (i > 3 && i < 6)
            {
                connection.signalGroupId = 2;
                connection.intersectionId = 1234;
                connection.signalGroupId_present = true;
                connection.connectingLane_laneId = connectingLane_laneId[i];
                adjacent_lanes[i].connections.push_back(connection);
            }
            else if (i > 11)
            {
                connection.signalGroupId = 3;
                connection.intersectionId = 1234;
                connection.signalGroupId_present = true;
                adjacent_lanes[i].connections.push_back(connection);
            }    

        for (int j = 0; j < list_xy[i].size(); j++)
        {
            geometry_msgs::Point xyz;
            xyz.x = list_xy[i][j][0];
            xyz.y = list_xy[i][j][1];
            xyz.z = 0.0; 
            adjacent_lanes[i].lane_coordinates.push_back(xyz);
        }

        intersections.adjacent_lanes.push_back(adjacent_lanes[i]);
    }

    intersections.id = 1234;
    intersections.refPoint_x = 0.0;
    intersections.refPoint_y = 0.0;
    intersections.refPoint_z = 0.0;
    intersections.maxPointX = 0.0;
    intersections.maxPointY = 0.0;
    intersections.minPointX = 0.0;
    intersections.minPointY = 0.0;

    //Main MAP
    mapout.intersections.push_back(intersections);
    mapout.layerIDs.push_back(-1);

    return mapout;

}


// Main function
// ==================================================================
int map_generator::init(int argc, char **argv)
{

    // Enable Parameter Support in Nodehandle
    ros::NodeHandle n("~");

	// Get General Parameters
	n.param<double>("config/frequency", frequency, 1.0);
	n.param<std::string>("config/topic_out", topic_out, "/MAPEMs");

	ROS_INFO("Publish on topic: %s with %f Hz", topic_out.c_str(), frequency);

	n.param<std::string>("frame_id", frame_id, "");
	n.param<int>("id", id, -1);

	// Init loopRate
	ros::Rate loopRate(frequency);

	// Init publisher
	pub_map_ = n.advertise<definitions::v2x_MAP>(topic_out, 0);

    //initialize x,y coordinate of the first point of each lanes at intersection
     std::vector<std::vector<float>> xy{{-3, -5}, {-3, -2}, {3, 2}, {3, 5}, {1, -8}, {-1, 8}, {-3, 2}, {-3, 5}, {3, -5}, {3, -2}, {-1, -8}, {1, 8} };

    //initialize x,y coordinate of the first point of added traffic light
    std::vector<std::vector<float>> xy_add{{38.55738067626953, 29.113723754882812}};

    //initialize the incline angle of each added ingress lane
    std::vector<float> angle{31};

    //constant distance between each ingress/ergress lane points
    float d = 0.996;
   
    ROS_INFO_STREAM("total number of lanes at intersection: " << xy.size());
    ROS_INFO_STREAM("total number of added traffic lights: " << xy_add.size());


    //Creates 12 consecutive lane points based on the first point of each lanes at intersection
    std::vector<std::vector<std::vector<float>>> lane_coordinates = generate_lane_points(xy, d); 
    //transform all lane points at intersection from local to global coordinate 
    std::vector<std::vector<std::vector<float>>> list_gxy = local_to_glob_coordinates(lane_coordinates);
    //construct the ingress lane points for the new traffic lights
    std::vector<std::vector<std::vector<float>>> list_xy_add = get_coordinates_list(xy_add, d, angle); 

    //append the list_xy_add to list_gxy vector list
    for(int i = 0; i < list_xy_add.size(); i++)
    {
        list_gxy.push_back(list_xy_add[i]);
    }

    // Start main loop
    while(ros::ok())
    {

    	// Fill map message
    	definitions::v2x_MAP mapMessage = fillMap(list_gxy);

        // Publish map message
        pub_map_.publish(mapMessage);
        
    	// Sleep
    	loopRate.sleep();
    }

    // End process
    return 0;

}
}