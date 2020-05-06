#include <signal.h>
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
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

static const string COLOR_RED = "\e[0;31m";
static const string COLOR_GREEN = "\e[0;32m";
static const string COLOR_YELLOW = "\e[0;33m"; 
static const string COLOR_NC = "\e[0m";

class ReadCSV
{
    private:
        std::vector<double> matrix;
        
        bool read_file(char* file)
        {
            std::ifstream f;
            std::string line;
            f.open(file, std::ifstream::in);

            if(!f)
            {
                printf("Can't open the file!!!\n");
                return false;
            }

            while(std::getline(f, line))
            {
                std::istringstream templine(line);
                std::string data;
                while(std::getline(templine, data, ','))
                {
                    matrix.push_back(atof(data.c_str()));
                    //printf("%f ",atof(data.c_str()));
                }
                //printf("\n");
            }
            f.close();
            return true;
        }
    public:
        ReadCSV(char* file)
        {
            if(!read_file(file))
                return;
            
        }
};


int main(int argc, char** argv)
{
    if(argc!=2)
    {
        printf("Not enough input, please provide input file!!! \n");
        return -1;
    }
    printf("Start:\n");
    ReadCSV r_csv(argv[1]);
    printf("-----------------------------END!!!-----------------------------\n");
    return 0;
}