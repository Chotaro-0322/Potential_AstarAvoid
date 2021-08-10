#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
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
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

using namespace grid_map;


class Cal_potential{
public:
    void run();
    void lidar_callback(const sensor_msgs::PointCloud2& data);
    ros::Publisher pub;
    ros::Publisher pub_map;
    ros::Subscriber sub;
};

void Cal_potential::run(){

    ros::NodeHandle nh;

    pub = nh.advertise<std_msgs::Float32MultiArray>("obj_potential_array", 10);
    pub_map = nh.advertise<grid_map_msgs::GridMap>("potential_obj", 1, true);

    sub = nh.subscribe("/points_no_ground", 10, &Cal_potential::lidar_callback, this);
    // sub = nh.subscribe("/points_raw", 10, &Cal_potential::lidar_callback, this);

    ros::Rate loop_rate(1);

    while (ros::ok()){
        ros::spinOnce();
    }
}

void Cal_potential::lidar_callback(const sensor_msgs::PointCloud2& data){
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
    passthrough_filter.setFilterLimits(0.7, 1.5); //other position
    // passthrough_filter.setFilterLimitsNegative (false);
    // passthrough_filter.filter (*cloud);
    
    // KD木を作っておく。近傍点探索とかが早くなる。
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    // std::cout << cloud->points[:] << std::endl;

    //近傍点探索に使うパラメータと結果が入る変数
    double radius = 3;  //半径r
    std::vector<int> k_indices;  //範囲内の点のインデックスが入る
    std::vector<float> k_sqr_distances;  //範囲内の点の距離が入る
    unsigned int max_nn = 0;  //何点見つかったら探索を打ち切るか。0にすると打ち切らない
    
    pcl::PointXYZ p;  //中心座標
    // p.x = 0.0;
    // p.y = 0.0;
    // p.z = 0.0;

    // //半径r以内にある点を探索
    // tree->radiusSearch(p, radius, k_indices, k_sqr_distances, max_nn);
    
    // if(k_indices.size() == 0) return;
    
    // pcl::PointXYZ result = cloud->points[k_indices[0]];

    // std::cout << "points size :" << k_sqr_distances.size() << std::endl;
    // for (auto i = 0; i < k_indices.size(); i++){
    //     std::cout << k_indices[i] << std::endl;
    // }
    // std::cout << "\n" << std::endl;
    // ROS_INFO("A nearest point of (0.5, 0.5) is...\nx: %lf, y:%lf, z:%lf", result.x, result.y, result.z);
  
    // std::cout << result.x << result.y << result.z << std::endl;q

    std::vector<std::vector<float>> potential_array(200, std::vector<float>(200, 0));

    for (float k_y = -10; k_y < 10; k_y += 0.1){
        for (float k_x = -10; k_x < 10; k_x += 0.1){
            p.x = k_x;
            p.y = k_y;
            p.z = 0.7;

            tree->radiusSearch(p, radius, k_indices, k_sqr_distances, max_nn);
            
            if(k_indices.size() == 0) continue;

            // pcl::PointXYZ result = cloud->points[k_indices[0]];

            for (int i = 0; i < k_indices.size(); i++){
                int x = (k_x + 10)*10;
                int y = (k_y + 10)*10;
                // std::cout << "x is " << x << std::endl;
                potential_array.at(y).at(x) += (1 / k_sqr_distances.at(i));
            }
        }
    }

    // 正規化
    float maxi = 0;
    float mini = 1000;
    for (int i = 0; i < potential_array.size(); ++i){
        maxi = std::max(maxi, (float)*std::max_element(potential_array[i].begin(), potential_array[i].end()));
        mini = std::min(mini, (float)*std::min_element(potential_array[i].begin(), potential_array[i].end()));
    }

    std::cout << "max :" << maxi << std::endl;
    std::cout << "min :" << mini << std::endl;

    for (float k_y = -10; k_y < 10; k_y += 0.1){
        for (float k_x = -10; k_x < 10; k_x += 0.1){
            int x = (k_x + 10)*10;
            int y = (k_y + 10)*10;
            potential_array.at(y).at(x) = (potential_array.at(y).at(x) - mini) / (maxi - mini);
            // std::cout << potential_array.at(y).at(x) << std::endl;
        }
    }

    std_msgs::Float32MultiArray potential_rosarray;
    // std::cout << potential_array.size() << std::endl;
    // std::cout << potential_array.at(0).size() << std::endl;
    potential_rosarray.layout.dim.push_back(std_msgs::MultiArrayDimension());
    potential_rosarray.layout.dim.push_back(std_msgs::MultiArrayDimension());
    potential_rosarray.layout.dim[0].label = "height";
    potential_rosarray.layout.dim[1].label = "width"; potential_rosarray.layout.dim[0].size = potential_array.size(); potential_rosarray.layout.dim[1].size = potential_array.at(0).size(); potential_rosarray.layout.dim[0].stride = potential_array.size()*potential_array.at(0).size(); potential_rosarray.layout.dim[1].stride = potential_array.at(0).size(); potential_rosarray.layout.data_offset = 0; std::vector<float> send_vec(potential_array.size() * potential_array.at(0).size(), 0); for (int i = 0; i < potential_array.size(); i++){ for (int j = 0; j < potential_array.at(0).size(); j++){ send_vec[i*potential_array.at(0).size() + j] = potential_array[i][j]; } } potential_rosarray.data = send_vec; // std::cout << potential_rosarray.data.size() << std::endl; // std::cout << potential_rosarray.data.at(0).size() << std::endl; // potential_rosarray.data = potential_array; this->pub.publish(potential_rosarray);
    // Create grid map.
    GridMap map({"elevation"});
    map.setFrameId("map");
    int y_range = potential_array.size();
    int x_range = potential_array.at(0).size();
    // map.setGeometry(Length(x_range, y_range), 1);
    map.setGeometry(Length(x_range, y_range), 1);
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
        map.getLength().x(), map.getLength().y(),
        map.getSize()(0), map.getSize()(1));
    ros::Time time = ros::Time::now();
    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
      // Position position;
      // map.getPosition(*it, position);
      // map.at("elevation", *it) = -1.04 + 0.2 * std::sin(3.0 * 50.0 + 5.0 * position.y()) * position.x();
      // std::cout << it.getUnwrappedIndex()(0) << std::endl;
      int sampling = 1;
      if (it.getUnwrappedIndex()(1) * sampling < potential_array.size() && it.getUnwrappedIndex()(0) * sampling < potential_array[1].size()){
        map.at("elevation", *it) = potential_array.at(it.getUnwrappedIndex()(1)*sampling).at(it.getUnwrappedIndex()(0)*sampling);
        // map.at("elevation", *it) = doc["potential_U"].at(position.y()).at(position.x());
      }
    }
    // Publish grid map.
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    pub_map.publish(message);
    ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
}



int main(int argc, char** argv){
    ros::init(argc, argv, "potential_cal");
    Cal_potential cal_potential;
    cal_potential.run();

    return 0;
}