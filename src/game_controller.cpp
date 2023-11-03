#include "game_controller.hpp"

namespace {
using std::placeholders::_1;

constexpr std::string_view NODE_NAME = "game_controller";
constexpr std::string_view SUB_TOPIC = "ros_game/keyboard_down";
constexpr std::string_view PUB_TOPIC = "ros_game/player";
constexpr auto QUEUE_SIZE = 1;

constexpr auto PLAYER_SPEED = 5;
constexpr auto PLAYER_START_X = 375;
constexpr auto PLAYER_START_Y = 280;

void publishPlayerData(rclcpp::Publisher<ros_game::msg::ObjectData>::SharedPtr publisher, const Object& data) {
    ros_game::msg::ObjectData msg;
    msg.x = data.x;
    msg.y = data.y;
    publisher->publish(msg);
}
}  // namespace

GameController::GameController() : Node(NODE_NAME.data()), player_{PLAYER_START_X, PLAYER_START_Y} {
    subscription_ = this->create_subscription<std_msgs::msg::Char>(
        SUB_TOPIC.data(), QUEUE_SIZE, std::bind(&GameController::gameUpdateCallback, this, _1));
    publisher_ = this->create_publisher<ros_game::msg::ObjectData>(PUB_TOPIC.data(), QUEUE_SIZE);

    publishPlayerData(publisher_, player_);
}

GameController::~GameController() {}

void GameController::gameUpdateCallback(const std_msgs::msg::Char& msg) {
    switch (msg.data) {
        case 'w':
            player_.y -= PLAYER_SPEED;
            break;
        case 's':
            player_.y += PLAYER_SPEED;
            break;
        case 'a':
            player_.x -= PLAYER_SPEED;
            break;
        case 'd':
            player_.x += PLAYER_SPEED;
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Received unknown key: %c", msg.data);
            break;
    }

    publishPlayerData(publisher_, player_);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GameController>());
    rclcpp::shutdown();
    return 0;
}
