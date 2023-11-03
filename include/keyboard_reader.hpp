#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/char.hpp"

class KeyboardReader final : public rclcpp::Node {
   public:
    KeyboardReader();
    ~KeyboardReader();

    bool handleEvents();

   private:
    rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr publisher_;
};
