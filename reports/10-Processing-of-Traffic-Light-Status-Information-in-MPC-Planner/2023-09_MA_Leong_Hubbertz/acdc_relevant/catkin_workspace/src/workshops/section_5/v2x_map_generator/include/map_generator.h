#include <string>
#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <cmath>

#include <definitions/v2x_MAP.h>
#include <definitions/v2x_MAP_Intersection.h>
#include <definitions/v2x_MAP_Lane.h>
#include <definitions/v2x_MAP_Connection.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#define PI 3.14159265

namespace map_node {

class map_generator{
protected:

  ros::Publisher pub_map_;

  int epoch_01_2022_ = 1640995200;

  // Params
  double frequency;
  std::string topic_out;

  // Custom MAPEM
  std::string frame_id;
  int id;
  
public:
  map_generator();

  ~map_generator();

  int init(int argc, char **argv);

  definitions::v2x_MAP fillMap(std::vector<std::vector<std::vector<float>>> list_xy);
  std::vector<std::vector<std::vector<float>>> generate_lane_points(std::vector<std::vector<float>> xy, float d); 
  std::vector<std::vector<std::vector<float>>> local_to_glob_coordinates(std::vector<std::vector<std::vector<float>>> list_xy);
  std::vector<std::vector<std::vector<float>>> get_coordinates_list(std::vector<std::vector<float>> x_y_add, float d, std::vector<float> angle); 

};
}