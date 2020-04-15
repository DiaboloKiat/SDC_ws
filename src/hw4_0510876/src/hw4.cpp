/*
    Reference:
        https://en.wikipedia.org/wiki/Extended_Kalman_filter (EKF)
        http://wiki.ros.org/robot_pose_ekf (robot_pose_ekf)
    
    Plot the odometry data, it is in red for /zed/odom, for fusioned one, it is in green. Only consider 
    position data
    
    Subscribe topics:
        /zed/odom (nav_msgs/Odometry): raw data from zed
        /robot_pose_ekf/odom_combined (geometry_msgs::PoseWithCovarianceStamped): after fusion visual odometry and IMU data
    
    Publish topics:
        /visualize/visual_odometry (visualization_msgs/Marker): corresponding to /zed/odom
        /visualize/combined_odometry (visualization_msgs/Marker): corresponding to /robot_pose_ekf/odom_combined
*/

#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <std_msgs/ColorRGBA.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>

class HW4
{
    private:
        //Node handler
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        //Publisher and Subscriber
        ros::Publisher pub_vo_path, pub_co_path, pub_imu_path, pub_imu_;
        ros::Subscriber sub_vo_path, sub_co_path, sub_imu_path;
        
        visualization_msgs::Marker visual_odometry_, combined_odometry_, imu_integration_;
        std_msgs::ColorRGBA r, g, b;
        bool first_data_; // First data recieved

        // Parameters from ROS server
        bool two_d_mode_;
        ros::Time last_time_; // Last timestamp
    
        //Eigen
        Eigen::Matrix3f C = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f imu2cam_trans, cam2zed_trans;
        Eigen::Vector3f gravity;
        Eigen::Vector3f v_g, s_g;

        /*
            Initial marker, including action, type, pose, scale and color
            int color: 1 =>  visual_odometry(red), 2 => combined_odometry(green), 3 => imu_integration(blue)
        */
        void initial_marker(visualization_msgs::Marker &msg, int color)
        {
            msg.action = visualization_msgs::Marker::ADD;
            msg.type = visualization_msgs::Marker::LINE_STRIP;
            msg.pose.orientation.w = 1.0;
            msg.scale.x = msg.scale.y = msg.scale.z = 0.02; // 2 cm
            if(color == 1)
                msg.color = r;
            else if(color == 2)
                msg.color = g;
            else
                msg.color = b;
        }

        // Callback for /zed/odom
        void cb_visual_odometry(const nav_msgs::Odometry msg)
        {
            visual_odometry_.header.frame_id = "marker"; // Use "marker" as filtered one to plot in same global frame
            geometry_msgs::Point p;
            p.x = msg.pose.pose.position.x;
            p.y = msg.pose.pose.position.y;
            p.z = msg.pose.pose.position.z;
            visual_odometry_.points.push_back(p);
            pub_vo_path.publish(visual_odometry_);
        }

        // Callback for /robot_pose_ekf/odom_combined
        void cb_combined_odometry(const geometry_msgs::PoseWithCovarianceStamped msg)
        {
            combined_odometry_.header.frame_id = msg.header.frame_id;
            geometry_msgs::Point p;
            p.x = msg.pose.pose.position.x;
            p.y = msg.pose.pose.position.y;
            p.z = msg.pose.pose.position.z;
            combined_odometry_.points.push_back(p);
            pub_co_path.publish(combined_odometry_);
        }

        //Publish imu with given position
        void pub_imu(double x, double y, double z = 0)
        {
            imu_integration_.header.frame_id = "marker";
            geometry_msgs::Point p;
            p.x = x; 
            p.y = y;
            if(!two_d_mode_) 
                p.z = z;
            imu_integration_.points.push_back(p);
            pub_imu_path.publish(imu_integration_);
        }

        //Callback for /imu/data
        //If first data, save the acceleration as gravity
        //Else, update C matrix and integrate velocity and position vector 
        //and publish pub_imu_path
        void cb_imu(const sensor_msgs::Imu msg)
        {
            imu2zedOdom(msg);
            if(first_data_){
                first_data_ = false; // Update flag
                
                // Using first acceleration as gravity
                gravity[0] = msg.linear_acceleration.x;
                gravity[1] = msg.linear_acceleration.y;
                gravity[2] = msg.linear_acceleration.z;   
            }
            else{
                // Update C
                double dt = (msg.header.stamp - last_time_).toSec();
                Eigen::Vector3f w = Eigen::Vector3f(msg.angular_velocity.x*dt, 
                                                    msg.angular_velocity.y*dt, 
                                                    msg.angular_velocity.z*dt);
                Eigen::Matrix3f B, B_square;
                B << 0, -w[2], w[1],
                     w[2], 0, -w[0],
                     -w[1], w[0], 0; // w_bz&t => w[2]; w_by&t => w[1]; w+bx&t => w[0]  (37)
                
                B_square = B*B;
                double sigma = w.norm();
                Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
                C = C*(I+(sin(sigma)/sigma)*B+((1-cos(sigma))/(sigma*sigma))*B_square); // (41)
                
                // Update v and s
                Eigen::Vector3f a_b, a_g;
                a_b[0] = msg.linear_acceleration.x, 
                a_b[1] = msg.linear_acceleration.y, 
                a_b[2] = msg.linear_acceleration.z;
                a_g = C * a_b; // (42)
                v_g += dt*(a_g-gravity); // (43)
                s_g += dt*v_g; // (44)
            }

            Eigen::Vector3f s_g_tf = cam2zed_trans * imu2cam_trans * s_g;

            if(two_d_mode_) 
                pub_imu(s_g_tf[0], s_g_tf[1]);
            else 
                pub_imu(s_g_tf[0], s_g_tf[1], s_g_tf[2]);

            last_time_ = msg.header.stamp;
        }

        //To transform /imu/data from IMU's frame to ZED odometry's frame
        void imu2zedOdom(const sensor_msgs::Imu msg)
        {
            sensor_msgs::Imu pub_msg;
            Eigen::Quaternionf imu_orientation_Q(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
            Eigen::Vector3f imu_angular(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
            Eigen::Vector3f imu_linear(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    
            Eigen::Matrix3f imu_orientation_M(imu_orientation_Q);
            Eigen::Matrix3f zed_orientation_M(cam2zed_trans * imu2cam_trans * imu_orientation_M);
            Eigen::Vector3f zed_angular(cam2zed_trans * imu2cam_trans * imu_angular);
            Eigen::Vector3f zed_linear(cam2zed_trans * imu2cam_trans * imu_linear);
            Eigen::Quaternionf zed_orientation_Q(zed_orientation_M);

            pub_msg.header = msg.header;
            pub_msg.orientation.w = zed_orientation_Q.w();
            pub_msg.orientation.x = zed_orientation_Q.x();
            pub_msg.orientation.y = zed_orientation_Q.y();
            pub_msg.orientation.z = zed_orientation_Q.z();
            pub_msg.angular_velocity.x = zed_angular(0);
            pub_msg.angular_velocity.y = zed_angular(1);
            pub_msg.angular_velocity.z = zed_angular(2);
            pub_msg.linear_acceleration.x = zed_linear(0);
            pub_msg.linear_acceleration.y = zed_linear(1);
            pub_msg.linear_acceleration.z = zed_linear(2);

            pub_imu_.publish(pub_msg);
        }

    public:
        HW4(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh), first_data_(true)
        {
            // Get parameters
            pnh_.param<bool>("two_d_mode_", two_d_mode_, true);

            // Plot color
            r.r = r.a = g.g = g.a = b.b = b.a = 1.0;
  
            // Initial marker
            initial_marker(visual_odometry_, 1);
            initial_marker(combined_odometry_, 2);
            initial_marker(imu_integration_, 3);

            // Subscribers
            sub_vo_path = nh.subscribe("/zed/odom", 1, &HW4::cb_visual_odometry, this);
            sub_co_path = nh.subscribe("/robot_pose_ekf/odom_combined", 1, &HW4::cb_combined_odometry, this);
            sub_imu_path = nh.subscribe("/imu/data", 1, &HW4::cb_imu, this);

            // Publisher
            pub_vo_path = nh.advertise<visualization_msgs::Marker>("/visual_odometry", 1);
            pub_co_path = nh.advertise<visualization_msgs::Marker>("/combined_odometry", 1);
            pub_imu_path = nh.advertise<visualization_msgs::Marker>("/result_marker", 1);
            pub_imu_ = nh.advertise<sensor_msgs::Imu>("/imu_data", 1);

            imu2cam_trans << 0.0225226, 0.999745, 0.0017194, 
                             0.0648765, -0.00317777, 0.997888, 
                             0.997639, -0.0223635, -0.0649315;
            cam2zed_trans << 0, 0, 1, 
                            -1, 0, 0, 
                             0, -1, 0;

            v_g << 0, 0, 0;
            s_g << 0, 0, 0;
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_path_node");
    ros::NodeHandle nh, pnh("~");
    HW4 hw4(nh, pnh);

    while(ros::ok())
        ros::spinOnce();
    
    return 0;
}