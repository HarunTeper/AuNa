#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "etsi_its_msgs/msg/cam.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

// Robot transmitter node to send CAMs to Artery and OMNeT++
class omnetTX : public rclcpp::Node
{
    public:
        // Create subscribers to get robot data and publisher to send to Artery and OMNeT++
        omnetTX(std::string robot_name)
        : Node("omnetTX"),latitude(0.0),longitude(0.0),altitude(0.0),heading(0.0),speed(0.0),acceleration(0.0)
        {
            publisher = this->create_publisher<etsi_its_msgs::msg::CAM>("camTX", 10);

            timer = this->create_wall_timer(100ms, std::bind(&omnetTX::cam_callback, this));

            pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("localization_pose", 10, std::bind(&omnetTX::pose_callback, this, _1));
            odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&omnetTX::odom_callback, this, _1));

            this->robot_name = robot_name;
        }

    private:
        // Send CAM data to Artery and OMNeT++ with most recently received data
        void cam_callback()
        {
            etsi_its_msgs::msg::CAM msg;

            msg.header.stamp = this->get_clock()->now();
            msg.header.frame_id = robot_name;
            
            msg.its_header.message_id = msg.its_header.MESSAGE_ID_CAM;

            msg.station_type.value = msg.station_type.PASSENGER_CAR;

            // Scale car and adjust for data format
            msg.reference_position.longitude = this->longitude*10*10; //0.1m
            msg.reference_position.latitude = this->latitude*10*10; //0.1m
            msg.reference_position.altitude.value = this->altitude*100*10; //0.01m

            // Adjust for data format
            msg.high_frequency_container.heading.value = this->heading*10;//0.1 degree
            // Scale car and adjust for data format
            msg.high_frequency_container.speed.value = this->speed*100*10;//0.01 m/s
            msg.high_frequency_container.drive_direction.value = this->speed<0;
            // Adjust for data format
            msg.high_frequency_container.vehicle_length.value = 0.49*10;//m
            msg.high_frequency_container.vehicle_width.value = 0.18*10;//m
            // Scale car and adjust for data format
            msg.high_frequency_container.longitudinal_acceleration.value = this->acceleration*10*10;//0.1m/s^2
            // Scale car data format
            msg.high_frequency_container.curvature.value = this->curvature/10;//1/m
            // Adjust for data format
            msg.high_frequency_container.yaw_rate.value = this->yaw_rate*100;//0.01degree/s

            publisher->publish(msg);
        }

        // Receive and save the odometry data for speed, acceleration, yaw rate and curvature
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            float old_speed = this->speed;
            this->speed = sqrt(pow(msg->twist.twist.linear.x,2)+pow(msg->twist.twist.linear.y,2));//absolute speed times the sign of the direction
            this->acceleration = (old_speed-this->speed)/odom_rate;
            this->yaw_rate = msg->twist.twist.angular.z; //yaw rate in radians/s
            this->yaw_rate = this->yaw_rate * 180 / M_PI; //yaw rate in degree/s
            this->curvature = this->yaw_rate/std::max(0.01f,this->speed);
        }

        // Receive and save the robot pose data
        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            this->longitude = msg->pose.position.x;
            this->latitude = msg->pose.position.y;
            this->altitude = msg->pose.position.z;

            std::vector<double> rpy = euler_from_quaternion(std::vector<double>{msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w});

            this->heading = rpy[2]; // get heading in radians
            this->heading = this->heading * 180 / M_PI; // get heading in degree
            this->heading = this->heading - 360 * floor( this->heading / 360 ); // get heading in interval [0,360]
        }

        // Converts to Euler angles from quaternion representation of orientation
        std::vector<double> euler_from_quaternion(std::vector<double> quaternion)
        {
            double x = quaternion[0];
            double y = quaternion[1];
            double z = quaternion[2];
            double w = quaternion[3];

            double roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
            double pitch = asin(2 * (w * y - z * x));
            double yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

            return std::vector<double>{roll,pitch,yaw};
        }

        rclcpp::TimerBase::SharedPtr timer;

        rclcpp::Publisher<etsi_its_msgs::msg::CAM>::SharedPtr publisher;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;

        std::string robot_name;
        float latitude;
        float longitude;
        float altitude;
        float heading;
        float speed;
        float acceleration;
        float yaw_rate;
        float curvature;

        // Odometry rate for acceleration
        float odom_rate = 0.01;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // Set name for content of CAM message. Node and topic namespace set by launch file.
    if(argc > 1)
    {
        rclcpp::spin(std::make_shared<omnetTX>(argv[1]));
    }
    else
    {
        rclcpp::spin(std::make_shared<omnetTX>(""));
    }

    rclcpp::shutdown();
    return 0;
}