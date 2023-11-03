#include "game_display.hpp"

#include "SDL2/SDL.h"

namespace {
using std::placeholders::_1;

constexpr std::string_view NODE_NAME = "game_controller";
constexpr std::string_view SUB_TOPIC = "ros_game/player";
constexpr auto QUEUE_SIZE = 1;

constexpr std::string_view WINDOW_TITLE = "ROS Game";
constexpr auto WINDOW_WIDTH = 800;
constexpr auto WINDOW_HEIGHT = 600;

constexpr auto TANK_SIZE = 50;
constexpr auto CANNON_WIDTH = 10;
constexpr auto CANNON_HEIGHT = 40;

void drawTank(SDL_Renderer** renderer, int x, int y) {
    SDL_SetRenderDrawColor(*renderer, 0, 0, 0, 255);
    SDL_RenderClear(*renderer);

    // Draw tank body (square)
    SDL_SetRenderDrawColor(*renderer, 255, 0, 0, 255);
    SDL_Rect tankRect = {x, y, TANK_SIZE, TANK_SIZE};
    SDL_RenderFillRect(*renderer, &tankRect);

    // Draw cannon (rectangle)
    SDL_Rect cannonRect = {x + TANK_SIZE / 2 - CANNON_WIDTH / 2, y - CANNON_HEIGHT, CANNON_WIDTH, CANNON_HEIGHT};
    SDL_RenderFillRect(*renderer, &cannonRect);

    // Present the renderer
    SDL_RenderPresent(*renderer);
}
}  // namespace

GameDisplay::GameDisplay() : Node(NODE_NAME.data()), running_(true) {
    subscription_ = this->create_subscription<ros_game::msg::ObjectData>(
        SUB_TOPIC.data(), QUEUE_SIZE, std::bind(&GameDisplay::playerUpdateCallback, this, _1));

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        RCLCPP_ERROR(this->get_logger(), "SDL initialization failed: %s", SDL_GetError());
        exit(-1);
    }

    window_ = SDL_CreateWindow(WINDOW_TITLE.data(), SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WINDOW_WIDTH,
                               WINDOW_HEIGHT, SDL_WINDOW_SHOWN);
    if (!window_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create SDL window: %s", SDL_GetError());
        SDL_Quit();
        exit(-1);
    }

    renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create SDL renderer: %s", SDL_GetError());
        SDL_DestroyWindow(window_);
        SDL_Quit();
        exit(-1);
    }
}

GameDisplay::~GameDisplay() {
    if (renderer_) {
        SDL_DestroyRenderer(renderer_);
    }

    if (window_) {
        SDL_DestroyWindow(window_);
    }

    SDL_Quit();
}

void GameDisplay::startEventLoop() {
    while (running_) {
        handleEvents();
        rclcpp::spin_some(this->get_node_base_interface());
        SDL_Delay(100);
    }
}

void GameDisplay::handleEvents() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) {
            running_ = false;
        }
    }
}

void GameDisplay::playerUpdateCallback(const ros_game::msg::ObjectData& msg) { drawTank(&renderer_, msg.x, msg.y); }

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto game_display = std::make_shared<GameDisplay>();
    game_display->startEventLoop();
    rclcpp::shutdown();
    return 0;
}
