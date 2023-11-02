#include "game_display.hpp"

#include "SDL2/SDL.h"

namespace {
using std::placeholders::_1;

constexpr std::string_view NODE_NAME = "game_display";
constexpr std::string_view SUB_TOPIC = "ros_game/keyboard";
constexpr auto QUEUE_SIZE = 10;

constexpr std::string_view WINDOW_TITLE = "ROS Game";
constexpr auto WINDOW_WIDTH = 800;
constexpr auto WINDOW_HEIGHT = 600;
}  // namespace

GameDisplay::GameDisplay() : Node(NODE_NAME.data()), window_(this->get_logger()) {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        SUB_TOPIC.data(), QUEUE_SIZE, std::bind(&GameDisplay::gameUpdateCallback, this, _1));

    if (!window_.init(WINDOW_TITLE.data(), WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_SHOWN)) {
        exit(-1);
    }
}

GameDisplay::~GameDisplay() {}

void GameDisplay::gameUpdateCallback(const std_msgs::msg::String& msg) {
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg.data.c_str());
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GameDisplay>());
    rclcpp::shutdown();
    return 0;
}
