#include <SDL2/SDL.h>

#include "rclcpp/rclcpp.hpp"

class SDLWindow {
   public:
    SDLWindow(const rclcpp::Logger& logger);
    ~SDLWindow();

    bool init(const char* title, int width, int height, Uint32 flags);

    void render();

   private:
    SDL_Window* window_ = nullptr;
    SDL_Renderer* renderer_ = nullptr;
    const rclcpp::Logger& logger_;
};
