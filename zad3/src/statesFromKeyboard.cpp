#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <iostream>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("states_from_keyboard_node");
    auto publisher = node->create_publisher<std_msgs::msg::Int32>("int_topic", 10);

    while (rclcpp::ok()) {
        int input;
        std::cout << "Enter an integer from <1; 3> to publish state: ";
        std::cin >> input;

        if (std::cin.fail()||(input<0)||(input>3)) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            RCLCPP_WARN(node->get_logger(), "Invalid input. Please enter an integer between 1 and 3.");
            continue;
        }

        std_msgs::msg::Int32 msg;
        msg.data = input;

        RCLCPP_INFO(node->get_logger(), "Publishing: '%d'", msg.data);
        publisher->publish(msg);
        //rclcpp::Rate rate(10);
        //rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}