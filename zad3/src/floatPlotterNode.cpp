#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <deque>
#include <chrono>
#include <thread>

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

using std::placeholders::_1;
using namespace std::chrono_literals;

class FloatPlotter : public rclcpp::Node {
public:
    FloatPlotter() : Node("float_plotter") {
        subscription_ = this->create_subscription<std_msgs::msg::Float32>("float_topic", 10, std::bind(&FloatPlotter::topic_callback, this, _1));

        start_time_ = now();
    }

    void topic_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        double time_elapsed = (now() - start_time_).seconds();
        buffer_time_.push_back(time_elapsed);
        buffer_data_.push_back(msg->data);

        if (buffer_time_.size() > max_size_) {
            buffer_time_.pop_front();
            buffer_data_.pop_front();
        }
    }

    void plot_loop() {
        while (rclcpp::ok()) {
            std::vector<double> t(buffer_time_.begin(), buffer_time_.end());
            std::vector<double> y(buffer_data_.begin(), buffer_data_.end());

            plt::clf();  // Clear previous plot
            plt::xlim(std::max(0.0, t.empty() ? 0.0 : t.back() - 10), t.empty() ? 10 : t.back());
            plt::ylim(0, 10);  // Customize as needed
            plt::plot(t, y);
            plt::grid(true);
            plt::pause(0.1);
        }
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
    rclcpp::Time start_time_;

    std::deque<double> buffer_time_;
    std::deque<double> buffer_data_;
    const size_t max_size_ = 200;

    rclcpp::Time now() const {
        return this->get_clock()->now();
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FloatPlotter>();

    std::thread spin_thread([&]() { rclcpp::spin(node); });
    node->plot_loop();

    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}