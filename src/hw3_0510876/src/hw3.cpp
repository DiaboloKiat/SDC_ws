/*
    Dead Reckoning Using IMU Data
    Reference:  https://www.cl.cam.ac.uk/techreports/UCAM-CL-TR-696.pdf (Ch 6.1, 6.2)
                http://www.starlino.com/dcm_tutorial.html [direction cosine matrices (DCM)]
*/

#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

class HW3
{
    private:
        //Node handler
        ros::NodeHandle nh_, pnh_;

        //Publisher and Subscriber
        ros::Publisher pub_marker_;
        ros::Subscriber sub_imu_;

        visualization_msgs::Marker marker_;
        bool first_data_; // First data recieved
    
        // Parameters from ROS server
        bool two_d_mode_;
        ros::Time last_time_; // Last timestamp
    
        //Eigen
        Eigen::Matrix3f C = Eigen::Matrix3f::Identity(); 
        Eigen::Vector3f gravity;
        Eigen::Vector3f v_g, s_g;

        // Topic string
        const std::string SUB_STRING = "/imu/data";
        const std::string PUB_STRING = "/result_marker";
  
        // Initial marker with 
        //  Frame id: imu
        //  Draw type: line strip
        //  Scale: 0.02 meter
        //  Color: blue
        void init_marker(void)
        {
            marker_.header.frame_id = "imu";
            marker_.type = marker_.LINE_STRIP;
            marker_.action = marker_.ADD
            marker_.id = 0
            marker_.scale.x = 0.02; 
            //marker_.scale.y = 0.02; 
            //marker_.scale.z = 0.02; // 2 cm 
            marker_.color.r = 0; 
            marker_.color.g = 0; 
            marker_.color.b = 1; 
            marker_.color.a = 1; // Blue
        }

        //Publish marker with given position
        void pub_marker(double x, double y, double z = 0)
        {
            geometry_msgs::Point p;
            p.x = x; 
            p.y = y;
            if(!two_d_mode_) 
                p.z = z;
            marker_.points.push_back(p);
            pub_marker_.publish(marker_);
        }

        //Subscriber Callback
        //If first data, save the acceleration as gravity
        //Else, update C matrix and integrate velocity and position vector 
        //and publish marker
        void cbImu(sensor_msgs::Imu msg)
        {
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
                B << 0, -w[2], w[1], w[2], 0, -w[0], -w[1], w[0], 0; 
                B_square = B*B;
                double sigma = w.norm();
                Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
                C = C*(I+(sin(sigma)/sigma)*B+((1-cos(sigma))/(sigma*sigma))*B_square);
                // Update v and s
                Eigen::Vector3f a_b, a_g;
                a_b[0] = msg.linear_acceleration.x, 
                a_b[1] = msg.linear_acceleration.y, 
                a_b[2] = msg.linear_acceleration.z;
                a_g = C * a_b;
                v_g += dt*(a_g-gravity);
                s_g += dt*v_g;
            }

            if(two_d_mode_) 
                pub_marker(s_g[0], s_g[1]);
            else 
                pub_marker(s_g[0], s_g[1], s_g[2]);

            last_time_ = msg.header.stamp;
        }

    public:
        HW3(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh), first_data_(true)
        {
            // Get parameters
            pnh_.param<bool>("two_d_mode_", two_d_mode_, true);

            // Initial marker
            init_marker();

            // Setup publisher and subscriber
            pub_marker_ = nh.advertise<visualization_msgs::Marker>(PUB_STRING, 10);
            sub_imu_ = nh.subscribe(SUB_STRING, 10, &HW3::cbImu, this);
            v_g << 0, 0, 0;
            s_g << 0, 0, 0;
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle nh, pnh("~");
    HW3 hw3(nh, pnh);

    while(ros::ok())
        ros::spinOnce();
    
    return 0;
}