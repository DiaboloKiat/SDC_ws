#include <signal.h>
#include <fstream>
// C++ STL
#include <iostream>
#include <boost/filesystem.hpp> // Support in STL since C++14
#include <boost/date_time/posix_time/posix_time.hpp>
// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
// Eigen
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h> // RemoveNaN
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/registration/icp.h> 
#include <pcl/filters/passthrough.h>
// TF
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
// PCL_ROS
#include <pcl_ros/transforms.h>
// GeographicLib
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGBPtr;

#define kLatOrigin 24.7855644226
#define kLonOrigin 120.997009277
#define kAltOrigin 127.651
#define kGlobalFrame "map"

static const string COLOR_RED = "\e[0;31m";
static const string COLOR_GREEN = "\e[0;32m";
static const string COLOR_YELLOW = "\e[0;33m"; 
static const string COLOR_NC = "\e[0m";

class Solution1Node{
public:
    Solution1Node(ros::NodeHandle nh, ros::NodeHandle pnh);
    static void sigint_cb(int sig);
    void gnss_cb1(const sensor_msgs::NavSatFixConstPtr &msg);
    void gnss_cb2(const geometry_msgs::PointStampedConstPtr &msg);
    void pc_cb(const sensor_msgs::PointCloud2ConstPtr &msg);
    void imu_cb(const sensor_msgs::ImuConstPtr &msg);
    void map_setup(void);
    void filters_setup(double voxel_grid_size);
    void gps_marker_setup(void);
    ros::NodeHandle nh_, pnh_;
    ros::Publisher pub_gps_marker_;
    ros::Publisher pub_map_;        // whole map pointcloud
    ros::Publisher pub_submap_;     // sub map pointcloud
    ros::Publisher pub_result_pc_;     // sub map pointcloud
    ros::Publisher pub_local_pc_;   // local velodyne pointcloud
    ros::Publisher pub_init_pose_;
    ros::Subscriber sub_gnss_;
    ros::Subscriber sub_pc_;
    ros::Subscriber sub_imu_;

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_;  // Voxel grid filter
    PointCloudXYZPtr map_pc_ptr_;
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond_;
    pcl::ConditionalRemoval<pcl::PointXYZ> cond_removal_;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
    vector<PointCloudXYZPtr, Eigen::aligned_allocator<PointCloudXYZPtr> > clouds_list_;

    // GPS related
    GeographicLib::Geocentric earth_;
    GeographicLib::LocalCartesian proj_;
    int cnt_gps_msgs_;
    visualization_msgs::Marker line_strip_;
    bool flag_init_guess_;

    // IMU related
    int cnt_imu_msgs_;

    // TF related
    tf::TransformListener listener_;

    string map_name_;               // options: nctu, itri, nuscene
    string result_filename_;
    Eigen::Matrix4d guess_matrix_;
    
};


Solution1Node::Solution1Node(ros::NodeHandle nh, ros::NodeHandle pnh):
    nh_(nh), 
    pnh_(pnh), 
    cnt_gps_msgs_(0),
    cnt_imu_msgs_(0),
    flag_init_guess_(false),
    guess_matrix_(Eigen::Matrix4d::Identity(4, 4)) {

    // Signal callback
    signal(SIGINT, sigint_cb);

    // ROS parameters
    boost::posix_time::ptime posix_time(boost::posix_time::second_clock::local_time()); // for filename
    std::string time_str = boost::posix_time::to_iso_string(posix_time);
    double voxel_grid_size;

    ros::param::param<double>("~vg_size", voxel_grid_size, 0.25);
    ros::param::param<string>("~map_name", map_name_, "nctu");
    ros::param::param<string>("~result_name", result_filename_, map_name_ + "_" + time_str + ".csv");
    
    // ROS publisher, subscriber
    pub_map_ = nh.advertise<sensor_msgs::PointCloud2>("map_pc", 1);
    pub_submap_ = nh.advertise<sensor_msgs::PointCloud2>("submap_pc", 1);
    pub_local_pc_ = nh.advertise<sensor_msgs::PointCloud2>("local_pc_filtered", 1);
    pub_gps_marker_ = nh.advertise<visualization_msgs::Marker>("gps_marker", 1);
    pub_init_pose_ = nh.advertise<geometry_msgs::PoseStamped>("init_pose", 1);
    pub_result_pc_ = nh.advertise<sensor_msgs::PointCloud2>("result_pc", 1);
    
    sub_imu_ = nh_.subscribe("imu/data", 50, &Solution1Node::imu_cb, this);
    if(map_name_ == "nctu") { 
        // For nctu map, the GPS msg type is NavSatFix which need to be converted
        earth_ = GeographicLib::Geocentric(GeographicLib::Constants::WGS84_a(), \
                                        GeographicLib::Constants::WGS84_f());
        proj_ = GeographicLib::LocalCartesian(kLatOrigin, kLonOrigin, kAltOrigin);
        sub_gnss_ = nh.subscribe("fix", 50, &Solution1Node::gnss_cb1, this);
        sub_pc_ = nh_.subscribe("points_raw", 50, &Solution1Node::pc_cb, this);
    } else {

        sub_gnss_ = nh.subscribe("fix", 50, &Solution1Node::gnss_cb2, this);
        sub_pc_ = nh_.subscribe("lidar_points", 50, &Solution1Node::pc_cb, this);
    }

    // ICP constraints
    icp_.setMaximumIterations(1);
    icp_.setTransformationEpsilon(1e-9);
    icp_.setEuclideanFitnessEpsilon(1e-9);

    filters_setup(voxel_grid_size);
    map_setup();
    gps_marker_setup();
    
    cout << COLOR_GREEN << "solution1_node is ready." << COLOR_NC << endl;
}


void Solution1Node::sigint_cb(int sig) {
    cout << "\nNode name: " << ros::this_node::getName() << " is shutdown." << endl;
    ros::shutdown();
}


void Solution1Node::filters_setup(double voxel_grid_size) {
    // Voxel grid filter parameters setting
    voxel_grid_.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
    // Range condition filter parameters setting
    range_cond_ = pcl::ConditionAnd<pcl::PointXYZ>::Ptr(new pcl::ConditionAnd<pcl::PointXYZ> ());
    range_cond_->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
        new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, -1.5)));
    cond_removal_.setCondition(range_cond_);
    cond_removal_.setKeepOrganized(true);
}


void Solution1Node::map_setup(void) {
    // Find map pcd files
    string package_path = ros::package::getPath("midterm_localization");
    string maps_dir = package_path + "/../test_data/" + map_name_ + "_map";
    std::vector<std::string> maps_list;
    for(const auto &entry:boost::filesystem::directory_iterator(maps_dir)) {
        string map_file = entry.path().string();
        if(map_file.substr(map_file.find_last_of(".") + 1) == "pcd"){
            maps_list.push_back(map_file);
        }
    }

    // Load map pointcloud from pcd files 
    map_pc_ptr_ = PointCloudXYZPtr(new PointCloudXYZ);
    PointCloudXYZPtr tempPtr(new PointCloudXYZ);
    for(auto file_path: maps_list) {
        cout << file_path << endl;
        if(pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *tempPtr) == -1){
            ROS_ERROR("Cannot load map: %s, aborting...", file_path.c_str());
            ros::shutdown();
        } 
        *map_pc_ptr_ += *tempPtr;
    }
    ROS_INFO("Load all maps successfully, there are %d sub maps and %d points", \
            (int)maps_list.size(), \
            (int)map_pc_ptr_->points.size());

    // Remove nan
    std::vector<int> indice;
    pcl::removeNaNFromPointCloud(*map_pc_ptr_, *map_pc_ptr_, indice);

    // Downsampling
    voxel_grid_.setInputCloud (map_pc_ptr_);
    voxel_grid_.filter(*map_pc_ptr_);
    ROS_INFO("After voxel grid filtering, there are %d points", \
            (int)map_pc_ptr_->points.size());

    // Publish map pointcloud
    if(pub_map_.getNumSubscribers() > 0) {
        sensor_msgs::PointCloud2 map_msg;
        pcl::toROSMsg(*map_pc_ptr_, map_msg);
        map_msg.header.frame_id = kGlobalFrame;
        pub_map_.publish(map_msg);
    }
}


void Solution1Node::gps_marker_setup(void) {
    // GPS line strip
    line_strip_.header.frame_id = kGlobalFrame;
    line_strip_.ns = "linestrip";
    line_strip_.action = visualization_msgs::Marker::ADD;
    line_strip_.pose.orientation.w = 1.0;
    line_strip_.id = 1;
    line_strip_.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip_.scale.x = 0.4;
    line_strip_.color.r = 1.0;
    line_strip_.color.a = 1.0;
}

void Solution1Node::pc_cb(const sensor_msgs::PointCloud2ConstPtr &msg){
    PointCloudXYZPtr pc_raw(new PointCloudXYZ);
    PointCloudXYZPtr pc_filtered(new PointCloudXYZ);
    pcl::fromROSMsg(*msg, *pc_raw);

    // Apply conditional filter
    cond_removal_.setInputCloud(pc_raw);
    cond_removal_.filter (*pc_filtered);
    // Remove nan
    std::vector<int> indice;
    pcl::removeNaNFromPointCloud(*pc_filtered, *pc_filtered, indice);
    
    // Transform to base_link frame
    tf::StampedTransform tf_stamped;
    geometry_msgs::TransformStamped tf_msg;
    listener_.lookupTransform("/velodyne", "/base_link", ros::Time(0), tf_stamped);
    tf::transformStampedTFToMsg(tf_stamped, tf_msg);
    Eigen::Affine3d tf_eigen = Eigen::Affine3d::Identity();
    tf::transformMsgToEigen(tf_msg.transform, tf_eigen);
    pcl::transformPointCloud (*pc_filtered, *pc_filtered, tf_eigen);

    // Only publish topic when there are any subscriber
    if(pub_local_pc_.getNumSubscribers() > 0){
        sensor_msgs::PointCloud2 local_pc_msg;
        pcl::toROSMsg(*pc_filtered, local_pc_msg);
        local_pc_msg.header.frame_id = msg->header.frame_id;
        pub_local_pc_.publish(local_pc_msg);
    }

    clouds_list_.push_back(pc_filtered);
}


void Solution1Node::imu_cb(const sensor_msgs::ImuConstPtr &msg) {
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen(msg->orientation, q);
    Eigen::Matrix3d m_q;
    m_q = q.toRotationMatrix();

    // Get initial orientaion from IMU
    if(cnt_imu_msgs_ == 0) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j){
                guess_matrix_(i, j) = m_q(i, j);
            }
        }
    }

    cnt_imu_msgs_++;
}


void Solution1Node::gnss_cb2(const geometry_msgs::PointStampedConstPtr &msg) {
    // Get initial position from GPS
    if(cnt_gps_msgs_ == 0) {
        guess_matrix_(0, 3) = msg->point.x; 
        guess_matrix_(1, 3) = msg->point.y;
        guess_matrix_(2, 3) = msg->point.z;
    }

    // Set initial guessing pose once getting IMU data and GPS data
    if(cnt_imu_msgs_ > 0 && flag_init_guess_ == false) {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = msg->header.stamp;
        pose_msg.header.frame_id = kGlobalFrame;

        Eigen::Vector3d position = guess_matrix_.block(0, 3, 3, 1); // extract i,j,row,col
        tf::pointEigenToMsg(position, pose_msg.pose.position);

        Eigen::Matrix3d m_q;
        m_q = guess_matrix_.block(0, 0, 3, 3);
        Eigen::Quaterniond q(m_q);
        Eigen::Vector3d euler = m_q.eulerAngles(2, 1, 0);
        tf::quaternionEigenToMsg(q, pose_msg.pose.orientation);

        cout << "init_guess from GPS & IMU:" << endl;
        cout << "timestamp: " << pose_msg.header.stamp << endl;
        cout << "x,y,z: " << position(0) << ", " << position(1) << ", " << position(2) << endl;
        cout << "yaw, pitch, roll: " << euler(0) << ", " << euler(1) << ", " << euler(2) << endl;
        cout << "\n=============================================\n" << endl;
        pub_init_pose_.publish(pose_msg);
        flag_init_guess_ = true;

        ros::Time tt = pcl_conversions::fromPCL(clouds_list_.front()->header.stamp);
        cout << "first timestamp from LiDAR: " << tt << endl;
        if(pose_msg.header.stamp != tt) {
            cout << COLOR_RED << "ERROR: Lost first timestamp data, aborting...\n" << COLOR_NC << endl;
            exit(-1);
        }

    }

    // Continune show the GPS path
    line_strip_.points.push_back(msg->point);
    if(line_strip_.points.size()>15000)
        line_strip_.points.clear();
    pub_gps_marker_.publish(line_strip_);

    cnt_gps_msgs_++;
}

// Not be used in ITRI map 
void Solution1Node::gnss_cb1(const sensor_msgs::NavSatFixConstPtr &msg) {
    double x, y, z;

    // Warning: if the measurement of altitude is wrong,
    // then the result of projection will be wrong.
    proj_.Forward(msg->latitude, msg->longitude, kAltOrigin, x, y, z);

    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;

    line_strip_.points.push_back(p);
    if(line_strip_.points.size() > 15000)
        line_strip_.points.clear();

    pub_gps_marker_.publish(line_strip_);

    if(cnt_gps_msgs_ == 0) {
        cout << "lat: " << msg->latitude << endl;
        cout << "lon: " << msg->longitude << endl;
        cout << "alt: " << msg->altitude << endl;
    }

    cnt_gps_msgs_++;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "solution1_node");
    ros::NodeHandle nh, pnh("~");
    Solution1Node node(nh, pnh);
    
    // Create csv result file
    std::fstream file;
    string package_path = ros::package::getPath("midterm_localization") + "/";
    string result_root = package_path + "csv_files/";
    if(!boost::filesystem::exists(result_root))         // Check the directory
        boost::filesystem::create_directory(result_root);
    file.open(result_root + node.result_filename_, std::fstream::out);
    file << setiosflags(ios::fixed);

    ros::Rate rate(10);
    while(ros::ok()) {
        ros::spinOnce();

        if(!node.clouds_list_.empty() && node.flag_init_guess_ == true) {
            cout << "clouds list size: " << node.clouds_list_.size() << endl;
            PointCloudXYZPtr lidar_pc(new PointCloudXYZ);
            PointCloudXYZPtr submap_pc(new PointCloudXYZ);
            PointCloudXYZPtr result_pc(new PointCloudXYZ);
            
            // Get lidar pc from clouds_list
            lidar_pc = node.clouds_list_.front();
            node.clouds_list_.erase(node.clouds_list_.begin());

            // Extract sub map from guessed position
            Eigen::Vector3d tmp_position = node.guess_matrix_.block(0, 3, 3, 1);
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(node.map_pc_ptr_);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(tmp_position(0) - 40, tmp_position(0) + 40);
            pass.filter(*submap_pc);
            pass.setInputCloud(submap_pc);
            pass.setFilterFieldName("y");
            pass.setFilterLimits(tmp_position(1) - 40, tmp_position(1) + 40);
            pass.filter(*submap_pc);
            if(node.pub_submap_.getNumSubscribers() > 0) {
                PointCloudXYZRGBPtr pc_colored(new PointCloudXYZRGB);
                pcl::copyPointCloud(*submap_pc, *pc_colored);
                for(auto& point: *pc_colored) {
                    point.r = 0;
                    point.g = 255;
                    point.b = 0;
                }
                sensor_msgs::PointCloud2 map_msg;
                pcl::toROSMsg(*pc_colored, map_msg);
                map_msg.header.frame_id = kGlobalFrame;
                node.pub_submap_.publish(map_msg);
            }


            // ICP
            node.icp_.setInputSource(lidar_pc);
            node.icp_.setInputTarget(submap_pc); // node.map_pc_ptr_
            
            Eigen::Matrix4f guess = node.guess_matrix_.cast<float>();
            node.icp_.align(*result_pc, guess);
            cout << "ICP has converged:" << node.icp_.hasConverged() << 
                    " score: " << node.icp_.getFitnessScore() << std::endl;
            node.guess_matrix_ = node.icp_.getFinalTransformation().cast<double>();
            cout << "Result matrix :\n" << node.guess_matrix_ << endl;
            cout << "\n=============================================\n" << endl;
            if(node.pub_result_pc_.getNumSubscribers() > 0) {
                PointCloudXYZRGBPtr pc_colored(new PointCloudXYZRGB);
                pcl::copyPointCloud(*result_pc, *pc_colored);
                for(auto& point: *pc_colored) {
                    point.r = 255;
                    point.g = 0;
                    point.b = 0;
                }

                sensor_msgs::PointCloud2 map_msg;
                pcl::toROSMsg(*pc_colored, map_msg);
                map_msg.header.frame_id = kGlobalFrame;
                node.pub_result_pc_.publish(map_msg);
            }

            // Export result to CSV
            Eigen::Vector3d transition = node.guess_matrix_.block(0, 3, 3, 1); // extract i,j,row,col
            Eigen::Matrix3d m = node.guess_matrix_.block(0, 0, 3, 3);
            Eigen::Vector3d euler = m.eulerAngles(2, 1, 0); 

            ros::Time tt = pcl_conversions::fromPCL(lidar_pc->header.stamp);
            file << tt.toSec() << ",";
            file << transition(0) << "," << transition(1) << "," << transition(2) << ",";
            file << euler(0) << "," << euler(1) << "," << euler(2) << endl;
        }

        rate.sleep();
    }

    file.close();
    return 0;
}