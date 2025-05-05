#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <fcntl.h> // For STDIN_FILENO
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <thread>
#include <atomic>


//def setup_publishers(self):
//self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
//self.laser_scan_pub = self.create_publisher(LaserScan, 'scan', 10)
//self.tf_broadcaster = TransformBroadcaster(self)
//self.tf_static_broadcaster = StaticTransformBroadcaster(self)

//def setup_subscribers(self):
//self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)


class WasdTeleopNode : public rclcpp::Node {
public:
    WasdTeleopNode() : Node("wasd_teleop_node"), running_(true) {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        input_thread_ = std::thread(&WasdTeleopNode::keyboardLoop, this);
    }

    ~WasdTeleopNode() {
        running_ = false;
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    std::thread input_thread_;
    std::atomic<bool> running_;

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

    void keyboardLoop() {
        std::cout << "Control the robot using:\n";
        std::cout << "W = forward, S = backward, A = left, D = right, X = stop, Q = quit\n";

        while (running_) {
            char c = getch();
            geometry_msgs::msg::Twist msg;

            switch (c) {
                case 'w':
                case 'W':
                    msg.linear.x = 0.5;
                    break;
                case 's':
                case 'S':
                    msg.linear.x = -0.5;
                    break;
                case 'a':
                case 'A':
                    msg.angular.z = 1.0;
                    break;
                case 'd':
                case 'D':
                    msg.angular.z = -1.0;
                    break;
                case 'x':
                case 'X':
                    msg.linear.x = 0.0;
                    msg.angular.z = 0.0;
                    break;
                case 'q':
                case 'Q':
                    running_ = false;
                    return;
                default:
                    continue;
            }

            publisher_->publish(msg);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WasdTeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
