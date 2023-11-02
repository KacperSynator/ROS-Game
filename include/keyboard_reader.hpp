#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class KeyboardReader final : public rclcpp::Node {
   public:
    KeyboardReader();
    ~KeyboardReader();

    bool handleEvents();

   private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};
