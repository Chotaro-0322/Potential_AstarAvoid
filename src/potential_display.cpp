#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include <stdio.h>
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>

#include <string>

using namespace grid_map;
using json = nlohmann::json;

json read_json(std::string filename){
  std::ifstream ifs(filename);
  json j;
  ifs >> j;

  return j;
}

int main(int argc, char** argv)
{
  // read json file
  std::string filename = "/home/itolab-chotaro/HDD/Python/Potential_function/potential_value_itolab_lane1.json";

  json doc;
  doc = read_json(filename);

  // std::cout << doc["potential_xm"] << std::endl;
  // std::cout << doc["potential_xm"].size() << std::endl;
  // std::cout << doc["potential_ym"].at(1).size() << std::endl;

  // for (auto itr_y = doc["potential_xm"].begin(); itr_y != doc["potential_xm"].end(); ++itr_y){
  //     std::cout << itr_y << std::endl;
  // }

  // Initialize node and publisher.
  ros::init(argc, argv, "potential_grid_map");
  ros::NodeHandle nh("~");
  ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("potential_grid_map", 1, true);

  // Create grid map.
  GridMap map({"elevation"});
  map.setFrameId("map");
  int x_range = doc["potential_xm"].size();
  int y_range = doc["potential_xm"].at(0).size();
  // map.setGeometry(Length(x_range, y_range), 1);
  map.setGeometry(Length(x_range, y_range), 1);
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));

  // Work with grid map in a loop.
  ros::Rate rate(30.0);

  while (nh.ok()) {

    // Add data to grid map.
    ros::Time time = ros::Time::now();

    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
      // Position position;
      // map.getPosition(*it, position);
      // map.at("elevation", *it) = -0.04 + 0.2 * std::sin(3.0 * 50.0 + 5.0 * position.y()) * position.x();
      std::cout << it.getUnwrappedIndex()(1) << std::endl;
      int sampling = 3;
      if (it.getUnwrappedIndex()(1) * sampling < doc["potential_U"].size() && it.getUnwrappedIndex()(0) * sampling < doc["potential_U"][0].size()){
        map.at("elevation", *it) = doc["potential_U"].at(it.getUnwrappedIndex()(1)*sampling).at(it.getUnwrappedIndex()(0)*sampling);
        // map.at("elevation", *it) = doc["potential_U"].at(position.y()).at(position.x());
      }
    }

    // Publish grid map.
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    publisher.publish(message);
    ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

    // Wait for next cycle.
    rate.sleep();
  }

  return 0;
}