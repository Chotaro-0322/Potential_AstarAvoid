#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include <stdio.h>
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>


#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <vector>
#include <string>

using namespace grid_map;
using json = nlohmann::json;

json read_json(std::string filename){
  std::ifstream ifs(filename);
  json j;
  ifs >> j;

  return j;
}

class Display_pot{
public:
  void ROS_init();
  void run();
  void lidar_callback(const sensor_msgs::PointCloud2& data);
  void pose_callback(const geometry_msgs::PoseStamped& data);
  void get_std(std::vector<std::vector<float>>& array);
  json doc;
  std::vector<std::vector<float>> potential_array;
  ros::NodeHandle nh;
  double pose_x = 0;
  double pose_y = 0;
  ros::Publisher publisher;
  ros::Subscriber sub_c_pose;
  ros::Subscriber sub_lidar;
  float potential_max = 100;
  float potential_min = -100;
  float search_range_min = -5;
  float search_range_max = 5;

  // vectorのサイズ指定の初期化は以下のようにやる
  Display_pot() : potential_array(100, std::vector<float>(100, 0)){
    ROS_init();
  }
  // Display_pot() : nh("~"){}
};

void Display_pot::ROS_init(){
  // read json file
  std::string filename = "/home/chohome/potential_method/potential_value_itolab_lane1.json";

  doc = read_json(filename);

  // Initialize node and publisher.
  publisher = nh.advertise<grid_map_msgs::GridMap>("potential_grid_map", 1, true);
  sub_lidar = nh.subscribe("/points_no_ground", 1, &Display_pot::lidar_callback, this);
  // sub_lidar = nh.subscribe("/points_raw", 1, &Display_pot::lidar_callback, this);
  sub_c_pose = nh.subscribe("/current_pose", 1, &Display_pot::pose_callback,this);
  // ros::Subscriber subscriber = nh.subscribe("/obj_potential_array", 10, &Display_pot::potential_callback, this);
}

void Display_pot::run(){
  // Create grid map.
  GridMap map({"elevation"});
  map.setFrameId("map");
  int y_range = doc["potential_xm"].size();
  int x_range = doc["potential_xm"].at(0).size();

  float x_min = doc["potential_xm"].at(0).at(0);
  float x_max = doc["potential_xm"].at(0).back();
  float y_min = doc["potential_ym"].at(0).at(0);
  float y_max = doc["potential_ym"].back().at(0);

  std::cout << "x_range is " << x_range << std::endl;
  std::cout << "y_range is " << y_range << std::endl;


  // map.setGeometry(Length(x_range, y_range), 1);
  // map.setGeometry(Length(x_range, y_range), 1);
  map.setGeometry(Length(x_range, y_range), 1);
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));

  // Work with grid map in a loop.
  ros::Rate rate(100.0); 

  int init_x_min;
  int init_x_max;
  int init_y_min;
  int init_y_max;
  int x;
  int y;
  int sampling = 1;

  int count = 0;

  float maxi = 0;
  for (int i = 0; i < doc["potential_U"].size(); ++i){
      maxi = std::max(maxi, (float)*std::max_element(doc["potential_U"][i].begin(), doc["potential_U"][i].end()));
  }

  std::cout << "maxi :" << maxi << std::endl;

  while (nh.ok()) {

    ros::spinOnce();
    // Add data to grid map.
    ros::Time time = ros::Time::now();

    std::vector<std::vector<float>> tmp_array = doc["potential_U"];

    init_x_min = (pose_x -5 - x_min) * 10;
    init_x_max = (pose_x +5 - x_min) * 10;
    init_y_min = (pose_y -5  - y_min) * 10;
    init_y_max = (pose_y +5  - y_min) * 10;

    // std::cout << "init_x_min is " << init_x_min << std::endl;
    // std::cout << "init_y_min is " << init_y_min << std::endl;
    // std::cout << "tmp_array x is " << tmp_array.at(0).size() << std::endl;
    // std::cout << "tmp_array y is " << tmp_array.size() << std::endl;

    for (float k_y = -5; k_y < 5; k_y += 0.1){
      for (float k_x = -5; k_x < 5; k_x += 0.1){
        x = (k_x + 5)*10;
        y = (k_y + 5)*10;
        // std::cout << tmp_array.at(init_y_min + y).at(init_x_min + x) << std::endl;
        // std::cout << "init_x_min + x :" << init_x_min + x << std::endl;
        // std::cout << "init_y_min + y :" << init_y_min + y << std::endl;
        // std::cout << "tmp_array :" << tmp_array.at(init_x_min + x).at(init_y_min + y) << std::endl;
        // std::cout << "potential_array : " << potential_array.at(y).at(x) << std::endl;
        if (tmp_array.at(init_y_min + y).at(init_x_min + x) + (potential_array.at(y).at(x)) < maxi){
          tmp_array.at(init_y_min + y).at(init_x_min + x) += (potential_array.at(y).at(x));
          // tmp_array.at(init_y_min + y).at(init_x_min + x) = potential_array.at(y).at(x);
          // tmp_array.at(init_y_min + y).at(init_x_min + x) += 1;
        }else{
          tmp_array.at(init_y_min + y).at(init_x_min + x) = maxi;
        }
      }
    }

    // std::cout << "count : " << count << std::endl;
    count += 1;
    // マップの表示
    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
      // Position position;
      // map.getPosition(*it, position);
      // map.at("elevation", *it) = -1.04 + 0.2 * std::sin(3.0 * 50.0 + 5.0 * position.y()) * position.x();
      // std::cout << it.getUnwrappedIndex()(0) << std::endl;
      if (it.getUnwrappedIndex()(1) * sampling < tmp_array.size() && it.getUnwrappedIndex()(0) * sampling < tmp_array[1].size()){
        map.at("elevation", *it) = tmp_array.at(it.getUnwrappedIndex()(1)*sampling).at(it.getUnwrappedIndex()(0)*sampling);
        // map.at("elevation", *it) = doc["potential_U"].at(position.y()).at(position.x());
      }
    }
    // // Publish grid map.
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    publisher.publish(message);
    // ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
    // std::cout << "grid_map pose x : " << pose_x << std::endl;

    // Wait for next cycle.
    rate.sleep();
  }
}

void Display_pot::pose_callback(const geometry_msgs::PoseStamped& data){
  // std::cout << "In the pose callback!!" << std::endl;
  pose_x = std::round(data.pose.position.x * 10) / 10;
  pose_y = std::round(data.pose.position.y * 10) / 10;

  // std::cout << "pose x : " << pose_x << std::endl;
  
}

void Display_pot::lidar_callback(const sensor_msgs::PointCloud2& data){
  // std::cout << "In the lidar callback!!" << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nan (new pcl::PointCloud<pcl::PointXYZ>); // NaN値あり
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); // NaN値なし
  
  //sensor_msgs::PointCloud2からpcl::PointXYZに変換
  pcl::fromROSMsg(data, *cloud_nan);

  // NaN値が入ってるといろいろ面倒なので除去
  std::vector<int> nan_index;
  pcl::removeNaNFromPointCloud(*cloud_nan, *cloud, nan_index);

  // VoxelGridfilter
  pcl::VoxelGrid<pcl::PointXYZ> vg; 
  vg.setInputCloud (cloud);  
  vg.setLeafSize (0.1, 0.1, 0.1);  
  vg.filter (*cloud); 

  // z軸の指定した範囲で抽出
  pcl::PassThrough<pcl::PointXYZ> passthrough_filter;
  passthrough_filter.setInputCloud(cloud);
  passthrough_filter.setFilterFieldName("z");
  // passthrough_filter.setFilterLimits(-1.0, 0.1); // init for left
  passthrough_filter.setFilterLimits(0.7, 1.0); //other position
  // passthrough_filter.setFilterLimitsNegative (false);
  // passthrough_filter.filter (*cloud);
  
  // KD木を作っておく。近傍点探索とかが早くなる。
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
  int x;
  int y;
  if (cloud->points.size() != 0){
    tree->setInputCloud(cloud);
  
    // std::cout << cloud->points[:] << std::endl;

    //近傍点探索に使うパラメータと結果が入る変数
    double radius = 1;  //半径r
    std::vector<int> k_indices;  //範囲内の点のインデックスが入る
    std::vector<float> k_sqr_distances;  //範囲内の点の距離が入る
    unsigned int max_nn = 0;  //何点見つかったら探索を打ち切るか。0にすると打ち切らない
    
    pcl::PointXYZ p;  //中心座標
    // std::vector<std::vector<float>> potential_array(200, std::vector<float>(200, 0));      
    for (float k_y = search_range_min; k_y < search_range_max; k_y += 0.1){
        for (float k_x = search_range_min; k_x < search_range_max; k_x += 0.1){
            x = (k_x + search_range_max)*10;
            y = (k_y + search_range_max)*10;
            potential_array.at(y).at(x) = 0;
        }
    }

    for (float k_y = search_range_min; k_y < search_range_max; k_y += 0.1){
        for (float k_x = search_range_min; k_x < search_range_max; k_x += 0.1){
            p.x = k_x;
            p.y = k_y;
            p.z = 0.7;

            tree->radiusSearch(p, radius, k_indices, k_sqr_distances, max_nn);
            x = (k_x + search_range_max)*10;
            y = (k_y + search_range_max)*10;
            
            if(k_indices.size() == 0){
              potential_array.at(y).at(x)  = 0;
              continue;
            }

            // pcl::PointXYZ result = cloud->points[k_indices[0]];

            for (int i = 0; i < k_indices.size(); i++){
                potential_array.at(y).at(x) += (1 / k_sqr_distances.at(i));
                // std::cout << "Potential_array is " << potential_array.at(y).at(x) << std::endl;
            }

            if (potential_array.at(y).at(x) > potential_max){
              potential_array.at(y).at(x) = potential_max;
            }else if (potential_array.at(y).at(x) < potential_min){
              potential_array.at(y).at(x) = potential_min;
            }
        }
    }
  }else{
    for (float k_y = search_range_min; k_y < search_range_max; k_y += 0.1){
        for (float k_x = search_range_min; k_x < search_range_max; k_x += 0.1){
            x = (k_x + search_range_max)*10;
            y = (k_y + search_range_max)*10;
            potential_array.at(y).at(x) = potential_min;
        }
    }
  }
  // std::cout << "in the cloud->points " << std::endl;
  Display_pot::get_std(potential_array);
  // 正規化
  // float maxi = 0;
  // float mini = 1000;
  // for (int i = 0; i < potential_array.size(); ++i){
  //     maxi = std::max(maxi, (float)*std::max_element(potential_array[i].begin(), potential_array[i].end()));
  //     mini = std::min(mini, (float)*std::min_element(potential_array[i].begin(), potential_array[i].end()));
  // }

  // // std::cout << "max :" << maxi << std::endl;
  // // std::cout << "min :" << mini << std::endl;

  // for (float k_y = search_range_min; k_y < search_range_max; k_y += 0.1){
  //     for (float k_x = search_range_min; k_x < search_range_max; k_x += 0.1){
  //       x = (k_x + search_range_max)*10;
  //       y = (k_y + search_range_max)*10;
  //       std::cout << "max :" << maxi << std::endl;
  //       std::cout << "min :" << mini << std::endl;
  //       std::cout << "potential_array : " << potential_array.at(y).at(x) << std::endl;
  //       potential_array.at(y).at(x) = (potential_array.at(y).at(x) - mini) / (maxi - mini);
  //       // potential_array.at(y).at(x) = 2;
  //       std::cout << potential_array.at(y).at(x) << std::endl;
  //   }
  // }
}

void Display_pot::get_std(std::vector<std::vector<float>>& data){
  float sum = 0;
  float ave = 0;
  float deviation = 0;
  float square = 0;
  float square_sum = 0;
  float variance = 0;
  float std_result = 0;
  int x;
  int y;
  // 平均を求める
  // std::cout << "In the get_std : " << std::endl;
  for (float k_y = search_range_min; k_y < search_range_max; k_y += 0.1){
      for (float k_x = search_range_min; k_x < search_range_max; k_x += 0.1){
        x = (k_x + search_range_max)*10;
        y = (k_y + search_range_max)*10;
        sum += data.at(y).at(x);
      }
  }
  ave = sum / std::pow(((float)search_range_max - (float)search_range_min) / 0.1 , 2);
  std::cout << "ave is  " << ave << std::endl;
  
  //分散を求める
  for (float k_y = search_range_min; k_y < search_range_max; k_y += 0.1){
      for (float k_x = search_range_min; k_x < search_range_max; k_x += 0.1){
        x = (k_x + search_range_max)*10;
        y = (k_y + search_range_max)*10;
        deviation = data.at(y).at(x) - ave;
        square = std::pow(deviation, 2);
        square_sum += square;
      }
  }
  variance = square_sum / std::pow(((float)search_range_max - (float)search_range_min) / 0.1 , 2);

  std_result = std::sqrt(variance);
  // std::cout << "std_result : " << std_result << std::endl;

  for (float k_y = search_range_min; k_y < search_range_max; k_y += 0.1){
      for (float k_x = search_range_min; k_x < search_range_max; k_x += 0.1){
        x = (k_x + search_range_max)*10;
        y = (k_y + search_range_max)*10;
        data.at(y).at(x) = ( data.at(y).at(x) - ave ) / std_result;
        // std::cout << data.at(y).at(x) << std::endl;
      }
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "add_potential");
  Display_pot display_pot;
  display_pot.ROS_init();
  display_pot.run();

  // ros::spin();

  return 0;
}