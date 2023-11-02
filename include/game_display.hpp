#include "rclcpp/rclcpp.hpp"
#include "sdl_window.hpp"
#include "std_msgs/msg/string.hpp"

class GameDisplay final : public rclcpp::Node {
   public:
    GameDisplay();
    ~GameDisplay();

    void gameUpdateCallback(const std_msgs::msg::String& msg);

   private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    SDLWindow window_;
};
