#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <vector>
// #include <cmath>

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
        // Publisher for /float_topic
        matplotlib_publisher = this->create_publisher<std_msgs::msg::Float32>("float_topic", 10);

        using namespace std::chrono_literals;
        timer = this->create_wall_timer( 100ms, std::bind(&StatesNode::motionCallback, this));
    }

private:
    void intCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if(init_intCallback){RCLCPP_INFO(this->get_logger(), "RCVNG /int_topic: %d", msg->data);}
        init_intCallback = false;
        state = msg->data; //state is an integer from <1; 3>
    }    

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if(init_scanCallback){RCLCPP_INFO(this->get_logger(), "RCVNG /scan");}
        init_scanCallback = false;
        ranges = msg->ranges; //data of distances
        //auto intensities = msg->intensities; //data of intensities
        //auto angle_min = msg->angle_min; //angle of the first ray
        //auto angle_max = msg->angle_max; //angle of the last ray
        //auto angle_increment = msg->angle_increment; //angle between rays
        time_increment = msg->time_increment; //time between measurements
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if(init_odomCallback){RCLCPP_INFO(this->get_logger(), "RCVNG /odom");}
        init_odomCallback = false;
        //auto pos = msg->pose.pose.position; //pos.x, pos.y, pos.z
        //auto ori = msg->pose.pose.orientation; //ori.x, ori.y, ori.z, ori.w
        //auto lin = msg->twist.twist.linear; //lin.x, lin.y, lin.z
        //auto ang = msg->twist.twist.angular; //ang.x, ang.y, ang.z
    }

    void publishCmdVel(float lin_x, float ang_z)
    {
        auto msg = geometry_msgs::msg::Twist();
        const float MAX_LINEAR_X = 1.0;
        const float MIN_LINEAR_X = -1.0;
        const float MAX_ANGULAR_Z = 2.0;
        const float MIN_ANGULAR_Z = -2.0;
        float tmplin_x = std::clamp(lin_x, MIN_LINEAR_X, MAX_LINEAR_X);
        float tmpang_z = std::clamp(ang_z, MIN_ANGULAR_Z, MAX_ANGULAR_Z);
        msg.linear.x = lin_x;
        msg.angular.z = ang_z;
        cmd_vel_publisher->publish(msg);
        if(tmplin_x != lin_x) {
            RCLCPP_INFO(this->get_logger(), "Lin_x out of range");
        }
        if(tmpang_z != ang_z) {
            RCLCPP_INFO(this->get_logger(), "Ang_z out of range");
        }
        if(count%100 == 0) {
            RCLCPP_INFO(this->get_logger(), "Lin_x: %f, Ang_z: %f", lin_x, ang_z);
        }
    }

    void publishCmdVelLinX(float lin_x){
        auto msg = geometry_msgs::msg::Twist();
        const float MAX_LINEAR_X = 1.0;
        const float MIN_LINEAR_X = -1.0;
        float tmplin_x = std::clamp(lin_x, MIN_LINEAR_X, MAX_LINEAR_X);
        msg.linear.x = lin_x;
        cmd_vel_publisher->publish(msg);
        if(tmplin_x != lin_x) {
            RCLCPP_INFO(this->get_logger(), "Lin_x out of range");
        }
    }

    void publishCmdVelAngZ(float ang_z){
        auto msg = geometry_msgs::msg::Twist();
        const float MAX_ANGULAR_Z = 2.0;
        const float MIN_ANGULAR_Z = -2.0;
        float tmpang_z = std::clamp(ang_z, MIN_ANGULAR_Z, MAX_ANGULAR_Z);
        msg.angular.z = tmpang_z;
        cmd_vel_publisher->publish(msg);
        if(tmpang_z != ang_z) {
            RCLCPP_INFO(this->get_logger(), "Ang_z out of range");
        }
    }

    void publishMatplotlib(float data){
        auto msg = std_msgs::msg::Float32();
        msg.data = data;
        matplotlib_publisher->publish(msg);
    }

    void motionCallback(){
        auto start_time = std::chrono::steady_clock::now();
        count++;

        if (state == 1) {
            if(edge_state_1 == false) {
                if (edge_state_0 == true) {
                    edge_state_0 = false;
                    std::cout << "State 0: LEFT" << std::endl;
                }
                if (edge_state_2 == true) {
                    edge_state_2 = false;
                    restore_terminal();
                    std::cout << "State 2: LEFT" << std::endl;
                }
                if (edge_state_3 == true) {
                    edge_state_3 = false;
                    std::cout << "State 3: LEFT" << std::endl;
                }
                edge_state_1 = true;
                std::cout << "State 1: STOP" << std::endl;
            }
            else {
                if ((linear_x - 0.05) > 0) {
                    linear_x -= 0.05;
                } else if ((linear_x + 0.05) < 0) {
                    linear_x += 0.05;
                } else {
                    linear_x = 0.0;
                }
                if ((angular_z - 0.05) > 0) {
                    angular_z -= 0.05;
                } else if ((angular_z + 0.05) < 0) {
                    angular_z += 0.05;
                } else {
                    angular_z = 0.0;
                }
                publishCmdVel(linear_x, angular_z);
            }

        } 
            
        else if (state == 2) {
            if(edge_state_2 == false) {
                if (edge_state_0 == true) {
                    edge_state_0 = false;
                    std::cout << "State 0: LEFT" << std::endl;
                }
                if (edge_state_1 == true) {
                    edge_state_1 = false;
                    std::cout << "State 1: LEFT" << std::endl;
                }
                if (edge_state_3 == true) {
                    edge_state_3 = false;
                    std::cout << "State 3: LEFT" << std::endl;
                }
                edge_state_2 = true;
                std::cout << "State 2: Manual control" << std::endl;
                std::cout << "Enter a command (w: forward, s: back, a: left, d: right, q: quit, x: stop): ";
                set_terminal_raw_mode();
            } 
            else {
                char c = getch_nonblock();
                switch (c) {
                    case 'w':
                        linear_x += 0.1;
                        break;
                    case 's':
                        linear_x -= 0.1;
                        break;
                    case 'a':
                        angular_z += 0.1;
                        break;
                    case 'd':
                        angular_z -= 0.1;
                        break;
                    case 'x':
                        linear_x = 0.0;
                        angular_z = 0.0;
                        break;
                    case 'q':
                        restore_terminal();
                        std::cout << "Exiting manual control" << std::endl;
                        return;
                    default:
                        break;
                }
                publishCmdVel(linear_x, angular_z);
            }
        }

        else if (state == 3) {
            if(edge_state_3 == false) {
                if (edge_state_0 == true) {
                    edge_state_0 = false;
                    std::cout << "State 0: LEFT" << std::endl;
                }
                if (edge_state_1 == true) {
                    edge_state_1 = false;
                    std::cout << "State 1: LEFT" << std::endl;
                }
                if (edge_state_2 == true) {
                    edge_state_2 = false;
                    std::cout << "State 2: LEFT" << std::endl;
                }
                edge_state_3 = true;
                std::cout << "State 3: Automatic control" << std::endl;
            }
            else {
                if (ranges[0] > distance && ranges[14] > distance + 0.3 && ranges[256] > distance + 0.3) {
                    if (ranges[33] > distance || ranges[237] > distance) {
                        if (ranges[0] <= ranges[33]) {
                            //msg1.angular.z = this->angularVelocity; DO NOTHING
                        }
                        else if (ranges[0] <= ranges[237]) {
                            angular_z = -(angular_z) + 1.0;
                        }
                    }

                    RCLCPP_INFO(this->get_logger(), "ranges[0] %f, distance: %f", ranges[0], distance);

                    turn = false;
                    indexLeft = ranges[ranges.size()/8];
                    indexRight = ranges[3*ranges.size()/8];

                    //msg1.linear_x = this->linearVelocity; //here
                    traveledDistance += linear_x * time_increment; //here
                    publishCmdVel(angular_z, linear_x); //here
                }
                else {
                    traveledDistance += 0.0 * time_increment;
                    publishCmdVelLinX(0.0);
                    turn = true;
                }

                if (turn)
                {
                    if (indexLeft > indexRight)
                    {
                        RCLCPP_INFO(this->get_logger(), "1st condition indexLeft %d, indexRight %d\n", indexLeft, indexRight);

                        traveledDistance += linear_x * time_increment;
                        publishCmdVelLinX(linear_x);

                        if (ranges[ranges.size()/4] >= distance + 0.1 || ranges[ranges.size() / 4] == 0) {
                            //msg1.angular.z = this->angularVelocity;
                            RCLCPP_INFO(this->get_logger(), "msg.ranges at 90°: %f", ranges[67]);
                            traveledDistance += linear_x * time_increment;
                            publishCmdVel(angular_z, linear_x);
                        }
                        else {
                            turn = false;
                            RCLCPP_INFO(this->get_logger(), "turn %d", turn);
                        }
                    }

                    if (indexLeft <= indexRight)
                    
                    {
                        RCLCPP_INFO(this->get_logger(), "2nd condition indexLeft %d, indexRight %d\n", indexLeft, indexRight);

                        angular_z = -(angular_z);
                        traveledDistance += linear_x * time_increment;
                        publishCmdVel(angular_z, linear_x);

                        if (ranges[3 * ranges.size() / 4] >= distance + 0.1 || ranges[3 * ranges.size() / 4] == 0){

                            angular_z = -(angular_z);
                            RCLCPP_INFO(this->get_logger(), "msg.ranges at elif 90°: %f", ranges[67]);

                            traveledDistance += linear_x * time_increment;
                            publishCmdVel(angular_z, linear_x);
                        }
                        else{
                            turn = false;
                        }
                    }
                }

                RCLCPP_INFO(this->get_logger(), "Celebration time: \"%d\"", state);

            }

        }

        else if (state == 0) {
            if(edge_state_0 == false) {
                if(edge_state_1 == true) {
                    edge_state_1 = false;
                    std::cout << "State 1: LEFT" << std::endl;
                }
                if(edge_state_2 == true) {
                    edge_state_2 = false;
                    restore_terminal();
                    std::cout << "State 2: LEFT" << std::endl;
                }
                if(edge_state_3 == true) {
                    edge_state_3 = false;
                    std::cout << "State 3: LEFT" << std::endl;
                }
                std::cout << "State 0: Hard stop" << std::endl;
                edge_state_0 = true;
                linear_x = 0.0;
                angular_z = 0.0;
            }
            else{
                publishCmdVel(linear_x, angular_z);
            }
            
        }

        auto end_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        if (elapsed_time > 100) { // Assuming the timer period is 100ms
            RCLCPP_WARN(this->get_logger(), "motionCallback is taking too long: %ld ms", elapsed_time);
        }
        
        // Update orientation
        theta += (angular_z * elapsed_time * 0.001);
        // Compute displacement
        dx = linear_x * std::cos(theta) * (double)elapsed_time*0.001;
        dy = linear_x * std::sin(theta) * (double)elapsed_time*0.001;
        // Update position
        //x += dx;
        //y += dy;
        // Update total distance traveled
        tmpdistance = std::sqrt(dx * dx + dy * dy);
        total_distance += tmpdistance;
        publishMatplotlib(total_distance);
    }    
    
    void set_terminal_raw_mode() {
        struct termios newt;
        // Get current terminal settings
        if (tcgetattr(STDIN_FILENO, &orig_termios) < 0) {
            perror("tcgetattr()");
            return;  // or handle the error more gracefully
        }
        // Copy current settings and modify
        newt = orig_termios;
        newt.c_lflag &= ~(ICANON | ECHO);  // Disable line buffering and echo
        // Apply new settings
        if (tcsetattr(STDIN_FILENO, TCSANOW, &newt) < 0) {
            perror("tcsetattr()");
            return;
        }
        // Set non-blocking input
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (flags < 0) {
            perror("fcntl(F_GETFL)");
            return;
        }
        if (fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) < 0) {
            perror("fcntl(F_SETFL)");
            return;
        }
    }


    void restore_terminal() {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
        fcntl(STDIN_FILENO, F_SETFL, 0); // Restore default blocking behavior
    }

    char getch_nonblock() {
        char buf = 0;
        if (read(STDIN_FILENO, &buf, 1) < 0) {
            buf = 0;
        }
        return buf;
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr int_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr matplotlib_publisher;
    rclcpp::TimerBase::SharedPtr timer;

    struct termios orig_termios;

    int state = 1;

    bool edge_state_0 = false;
    bool edge_state_1 = false;
    bool edge_state_2 = false;
    bool edge_state_3 = false;

    bool turn = false;

    float distance = 0.5;

    int indexLeft;
    int indexRight;
    
    float linear_x = 0.0;
    float angular_z = 0.0;

    float traveledDistance = 0.0;
    float time_increment = 0.0;

    std::vector<float> ranges;

    bool init_intCallback = true;
    bool init_scanCallback = true;
    bool init_odomCallback = true;

    long int count = 0;

    double x, y, dx, dy, theta; // Robot pose
    double total_distance, tmpdistance;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatesNode>());
    rclcpp::shutdown();
    //atexit(restore_terminal);
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

/*
    char getch_nonblock() {
        char buf = 0;
        struct termios old = {};
        // Get current terminal settings
        if (tcgetattr(STDIN_FILENO, &old) < 0) perror("tcgetattr()");
        struct termios newt = old;
        newt.c_lflag &= ~(ICANON | ECHO); // Disable canonical mode and echo
        if (tcsetattr(STDIN_FILENO, TCSANOW, &newt) < 0) perror("tcsetattr()");
        // Set non-blocking mode on stdin
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
        // Try to read a single char
        if (read(STDIN_FILENO, &buf, 1) < 0) {
            buf = 0; // No input
        }
        // Restore terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &old);
        fcntl(STDIN_FILENO, F_SETFL, flags); // Restore flags
        return buf;
    }

*/

/*
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
*/

/*
    void set_terminal_raw_mode() {
        struct termios orig_termios;
        struct termios newt;
        if (tcgetattr(STDIN_FILENO, &orig_termios) < 0) perror("tcgetattr()");
        newt = orig_termios;
        newt.c_lflag &= ~(ICANON | ECHO);
        if (tcsetattr(STDIN_FILENO, TCSANOW, &newt) < 0) perror("tcsetattr()");
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK); // Set non-blocking input
        //int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        //fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }
*/

            /*
             if (this->actualState == 3)
            {
                if (msg->ranges[0] > distance &&
                    msg->ranges[14] > distance + 0.3 &&
                    msg->ranges[256] > distance + 0.3)
                {
                    if (msg->ranges[33] > distance || msg->ranges[237] > distance)
                    {
                        if (msg->ranges[0] <= msg->ranges[33])
                        {
                            msg1.angular.z = this->angularVelocity;
                        }
                        else if (msg->ranges[0] <= msg->ranges[237])
                        {
                            msg1.angular.z = -this->angularVelocity + 1.0;
                        }
                    }

                    RCLCPP_INFO(this->get_logger(), "msg.ranges[0]: %f", msg->ranges[0]);
                    RCLCPP_INFO(this->get_logger(), "distance: %f", distance);

                    this->turn = false;
                    this->right = false;
                    this->left = false;

                    this->savedMsgLeft = msg->ranges[msg->ranges.size() / 8];
                    this->savedMsgRight = msg->ranges[3 * msg->ranges.size() / 8];

                    msg1.linear.x = this->linearVelocity;
                    this->totalDistance += msg1.linear.x * msg->time_increment;
                    this->publisher_->publish(msg1);
                }
                else
                {
                    msg1.linear.x = 0.0;
                    this->totalDistance += msg1.linear.x * msg->time_increment;
                    this->publisher_->publish(msg1);
                    this->turn = true;
                }

                if (this->turn)
                {
                    if (this->savedMsgLeft > this->savedMsgRight)
                    {
                        RCLCPP_INFO(this->get_logger(), "NEFUNKCNA SPROSTOST: %f", this->savedMsgLeft);
                        RCLCPP_INFO(this->get_logger(), "PRAVA NEFUNKCNA BLBOST: %f", this->savedMsgRight);

                        msg1.angular.z = this->angularVelocity;
                        this->totalDistance += msg1.linear.x * msg->time_increment;
                        this->publisher_->publish(msg1);

                        if (msg->ranges[msg->ranges.size() / 4] >= distance + 0.1 || msg->ranges[msg->ranges.size() / 4] == 0)
                        {
                            msg1.angular.z = this->angularVelocity;
                            RCLCPP_INFO(this->get_logger(), "msg.ranges at 90°: %f", msg->ranges[67]);
                            this->totalDistance += msg1.linear.x * msg->time_increment;
                            this->publisher_->publish(msg1);
                        }
                        else
                        {
                            this->turn = false;
                            RCLCPP_INFO(this->get_logger(), "THE STATE OF FALSE IS: %d", this->turn);
                        }
                    }

                    if (this->savedMsgLeft <= this->savedMsgRight)
                    {
                        RCLCPP_INFO(this->get_logger(), "NEFUNKCNA SPROSTOST V DRUHOM: %f", this->savedMsgLeft);
                        RCLCPP_INFO(this->get_logger(), "PRAVA NEFUNKCNA BLBOST TAK TAK: %f", this->savedMsgRight);

                        msg1.angular.z = -this->angularVelocity;
                        this->totalDistance += msg1.linear.x * msg->time_increment;
                        this->publisher_->publish(msg1);

                        RCLCPP_INFO(this->get_logger(), "RANGES NA TOM CO CHCEM: %f", msg->ranges[3 * msg->ranges.size() / 4]);

                        if (msg->ranges[3 * msg->ranges.size() / 4] >= distance + 0.1 || msg->ranges[3 * msg->ranges.size() / 4] == 0)
                        {
                            msg1.angular.z = -this->angularVelocity;
                            RCLCPP_INFO(this->get_logger(), "msg.ranges at elif 90°: %f", msg->ranges[67]);
                            this->totalDistance += msg1.linear.x * msg->time_increment;
                            this->publisher_->publish(msg1);
                        }
                        else
                        {
                            this->turn = false;
                        }
                    }
                }

                RCLCPP_INFO(this->get_logger(), "Celebration time: \"%d\"", this->actualState);
            }
            */

/*
                int size = ranges.size();
                vector<float> x_distance(size);
                vector<float> y_distance(size);
                for (int i = 0; i < size; i++) {
                    if (ranges[i] *here condition that filters*){
                        x_distance[i] = ranges[i] * cos(angle_min + i * angle_increment);
                        y_distance[i] = ranges[i] * sin(angle_min + i * angle_increment);
                    } else {
                        x_distance[i] = 0.0;
                        y_distance[i] = 0.0;
                    }
                    
                }
                for (int i = 0; i < size; i++) {
                }
                publishCmdVel(linear_x, angular_z);
*/
