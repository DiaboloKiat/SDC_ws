// C++ STL
#include <iostream>
#include <boost/filesystem.hpp> // Support in STL since C++17
// ROS
#include <ros/ros.h>
#include <ros/package.h>
// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h> // RemoveNaN
#include <pcl/kdtree/kdtree_flann.h> // KD tree
#include <pcl/filters/voxel_grid.h> // VG
#include <pcl/registration/icp.h> // ICP
#include <pcl/filters/passthrough.h> // passThrough
// TF
#include <tf/transform_broadcaster.h>
// MSG
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGBPtr;


class LoadMapNode{
public:
    LoadMapNode(ros::NodeHandle nh, ros::NodeHandle pnh);
    ros::NodeHandle nh_, pnh_;
    ros::Publisher pub_map_;

    PointCloudXYZPtr map_pc_ptr_;
};

LoadMapNode::LoadMapNode(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh) 
{
    // ROS publisher, subscriber
    pub_map_ = nh.advertise<sensor_msgs::PointCloud2>("map_pc", 1);

    // Find map pcd files
    string package_path = ros::package::getPath("midterm_localization");
    string maps_path = package_path.substr(0, package_path.size() - strlen("midterm_localization")) + "test_data/map";
    std::vector<std::string> maps_list;
    for(const auto &entry:boost::filesystem::directory_iterator(maps_path)) 
    {
        string map_file = entry.path().string();
        if(map_file.substr(map_file.find_last_of(".") + 1) == "pcd")
	{
            maps_list.push_back(map_file);
        }
    }

    // Load map pointcloud from pcd files 
    map_pc_ptr_ = PointCloudXYZPtr(new PointCloudXYZ);
    PointCloudXYZPtr tempPtr(new PointCloudXYZ);
    for(auto file_path: maps_list) 
    {
        cout << file_path << endl;
        if(pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *tempPtr) == -1)
	{
            ROS_ERROR("Cannot find %s, aborting...", file_path.c_str());
            ros::shutdown();
        } 
        *map_pc_ptr_ += *tempPtr;
    }
    ROS_INFO("Successfully load all map, there are %d points", (int)map_pc_ptr_->points.size());

    // Publish map pointcloud
    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*map_pc_ptr_, map_msg);
    map_msg.header.frame_id = "map";
    pub_map_.publish(map_msg);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "load_map_node");
    ros::NodeHandle nh, pnh("~");
    LoadMapNode node(nh, pnh);
    ros::spin();
    return 0;
}
