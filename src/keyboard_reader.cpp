#include "keyboard_reader.hpp"

#include <ncurses.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>


namespace {
constexpr std::string_view NODE_NAME = "keyboard_reader";
constexpr std::string_view PUB_TOPIC = "ros_game/keyboard";
constexpr auto QUEUE_SIZE = 10;
}  // namespace

KeyboardReader::KeyboardReader() : Node(NODE_NAME.data()) {
    publisher_ = this->create_publisher<std_msgs::msg::String>(PUB_TOPIC.data(), QUEUE_SIZE);
    initscr();             // Initialize ncurses
    raw();                 // Disable line buffering
    keypad(stdscr, TRUE);  // Enable special keys
    noecho();
}

KeyboardReader::~KeyboardReader() { endwin(); }

bool KeyboardReader::handleEvents() {
    int ch = getch();

    if (ch == ERR) {
        return true;
    }

    auto message = std_msgs::msg::String();
    switch (ch) {
        case 'a':
            message.data = "a";
            break;
        case 'w':
            message.data = "b";
            break;
        case 's':
            message.data = "s";
            break;
        case 'd':
            message.data = "d";
            break;
        case 'q':
            return false;  // Exit the loop if 'q' is pressed
        default:
            return true;
            break;
    }
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);

    return true;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto keyboard_reader = std::make_shared<KeyboardReader>();

    bool running = true;
    while (running) {
        running = keyboard_reader->handleEvents();

        rclcpp::spin_some(keyboard_reader);
    }

    rclcpp::shutdown();
    return 0;
}
