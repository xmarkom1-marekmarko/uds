#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <iostream>


class StatesNode : public rclcpp::Node
{
public:
    StatesNode() : Node("states_node")
    {
        // subscriber for /int_topic
        int_subscriber = this->create_subscription<std_msgs::msg::Int32>("int_topic", 10, std::bind(&StatesNode::intCallback, this, std::placeholders::_1));
        // Subscriber for /scan topic
        scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&StatesNode::scanCallback, this, std::placeholders::_1));
        // Subscriber for /odom topic
        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&StatesNode::odomCallback, this, std::placeholders::_1));
        // Publisher for /cmd_vel topic
        cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void intCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "RCVD /int_topic: %d", msg->data);
        auto state = msg->data; //state is an integer from <1; 3>
        
    }    

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "RCVD /scan");
        auto ranges = msg->ranges; //data of distances
        auto intensities = msg->intensities; //data of intensities
        auto angle_min = msg->angle_min; //angle of the first ray
        auto angle_max = msg->angle_max; //angle of the last ray
        auto angle_increment = msg->angle_increment; //angle between rays
        
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto pos = msg->pose.pose.position; //pos.x, pos.y, pos.z
        auto ori = msg->pose.pose.orientation; //ori.x, ori.y, ori.z, ori.w
        auto lin = msg->twist.twist.linear; //lin.x, lin.y, lin.z
        auto ang = msg->twist.twist.angular; //ang.x, ang.y, ang.z
        RCLCPP_INFO(this->get_logger(), "RCVD /odom");
        
    }

    void publishCmdVel(float linear_x, float angular_z)
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = linear_x;
        msg.angular.z = angular_z;
        cmd_vel_publisher_->publish(msg);
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr int_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;



    void motion(){
        std::thread motion_thread([this]() {
            rclcpp::Rate rate(10); // 10 Hz loop rate
            while (rclcpp::ok()) {

                if (state == 1) {
                    if ((linear_x - 0.1) > 0) {
                        linear_x -= 0.1;
                    } else if ((linear_x + 0.1) < 0) {
                        linear_x += 0.1;
                    } else {
                        linear_x = 0.0;
                    }

                    if ((angular_z - 0.1) > 0) {
                        angular_z -= 0.1;
                    } else if ((angular_z + 0.1) < 0) {
                        angular_z += 0.1;
                    } else {
                        angular_z = 0.0;
                    }

                    publishCmdVel(linear_x, angular_z);
                } 
                
                else if (state == 2) {
                    std::thread keyboard_thread([this]() {
                        char input;
                        std::cout << "Enter a command (w: forward, s: stop, a: left, d: right): ";
                        std::cin >> input;

                        while (running) {
                            char c = getch();

                            switch (c) {
                                case 'w':
                                    linear_x = 0.1;
                                    break;
                                case 's':
                                    linear_x = -0.1;
                                    break;
                                case 'a':
                                    angular_z = 1.0;
                                    break;
                                case 'd':
                                    angular_z = -1.0;
                                    break;
                                case 'x':
                                    linear_x = 0.0;
                                    angular_z = 0.0;
                                    break;
                                case 'q':
                                    running_ = false;
                                    return;
                                default:
                                    continue;
                            }
                            publishCmdVel(linear_x, angular_z);
                        }
                    });
                    keyboard_thread.detach(); // Detach the thread to run independently
                }

                else if (state == 3) {
                    linear_x = 0.0;
                    angular_z = 0.0;
                    publishCmdVel(linear_x, angular_z);
                }

                else if (state == 0) {
                    linear_x = 0.0;
                    angular_z = 0.0;
                    publishCmdVel(linear_x, angular_z);
                }

                else {
                    linear_x = 0.0;
                    angular_z = 0.0;
                    publishCmdVel(linear_x, angular_z);
                }
                
                rate.sleep();
            }
        });
        motion_thread.detach(); // Detach the thread to run independently
    }    
    
    char getch() {
        char buf = 0;
        struct termios old = {};
        if (tcgetattr(STDIN_FILENO, &old) < 0) perror("tcgetattr()");
        struct termios newt = old;
        newt.c_lflag &= ~(ICANON | ECHO);
        if (tcsetattr(STDIN_FILENO, TCSANOW, &newt) < 0) perror("tcsetattr()");
        if (read(STDIN_FILENO, &buf, 1) < 0) perror("read()");
        if (tcsetattr(STDIN_FILENO, TCSADRAIN, &old) < 0) perror("tcsetattr()");
        return buf;
    }

         

    float linear_x = 0.0;
    float angular_z = 0.0;
}
// Access the latest LaserScan message
//auto latest_scan = node_instance->getLatestScan();
//if (latest_scan) {
//    // Perform calculations using the ranges
//    float min_range = *std::min_element(latest_scan->ranges.begin(), latest_scan->ranges.end());
//    RCLCPP_INFO(node_instance->get_logger(), "Minimum range: %f", min_range);
//}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatesNode>());
    rclcpp::shutdown();
    return 0;
}

/*
sensor_msgs/msg/LaserScan.msg
# Single scan from a planar laser range-finder
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data
#std_msgs/Header header # timestamp in the header is the acquisition time of
# # the first ray in the scan.
# # in frame frame_id, angles are measured around
# # the positive Z axis (counterclockwise, if Z is up)
# # with zero angle being forward along the x axis
#float32 angle_min # start angle of the scan [rad]
#float32 angle_max # end angle of the scan [rad]
#float32 angle_increment # angular distance between measurements [rad]
#float32 time_increment # time between measurements [seconds] - if your scanner
# # is moving, this will be used in interpolating position
# # of 3d points
#float32 scan_time # time between scans [seconds]
#float32 range_min # minimum range value [m]
#float32 range_max # maximum range value [m]
#float32[] ranges # range data [m]
# # (Note: values < range_min or > range_max should be discarded)
#float32[] intensities # intensity data [device-specific units]. If your
# # device does not provide intensities, please leave
# # the array empty.
*/

/*
nav_msgs/msg/Odometry.msg
# This represents an estimate of a position and velocity in free space.
# The pose in this message should be specified in the coordinate frame given by header.frame_id
# The twist in this message should be specified in the coordinate frame given by the child_frame_id
# Includes the frame id of the pose parent.
std_msgs/Header header
# Frame id the pose points to. The twist is in this coordinate frame.
string child_frame_id
# Estimated pose that is typically relative to a fixed world frame.
geometry_msgs/PoseWithCovariance pose
# Estimated linear and angular velocity relative to child_frame_id.
geometry_msgs/TwistWithCovariance twist
*/