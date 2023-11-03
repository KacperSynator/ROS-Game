#include <SDL2/SDL.h>

#include "rclcpp/rclcpp.hpp"
#include "ros_game/msg/object_data.hpp"
#include "std_msgs/msg/string.hpp"

class GameDisplay final : public rclcpp::Node {
   public:
    GameDisplay();
    ~GameDisplay();

    void playerUpdateCallback(const ros_game::msg::ObjectData& msg);
    void startEventLoop();
    void handleEvents();

   private:
    SDL_Window* window_ = nullptr;
    SDL_Renderer* renderer_ = nullptr;
    rclcpp::Subscription<ros_game::msg::ObjectData>::SharedPtr subscription_;

    bool running_;
};
