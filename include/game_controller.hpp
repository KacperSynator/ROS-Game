#include "object.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_game/msg/object_data.hpp"
#include "std_msgs/msg/char.hpp"

class GameController final : public rclcpp::Node {
   public:
    GameController();
    ~GameController();

    void gameUpdateCallback(const std_msgs::msg::Char& msg);

   private:
    rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr subscription_;
    rclcpp::Publisher<ros_game::msg::ObjectData>::SharedPtr publisher_;
    Object player_;
};
