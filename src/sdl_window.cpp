#include "sdl_window.hpp"

#include <SDL2/SDL.h>


SDLWindow::SDLWindow(const rclcpp::Logger& logger) : logger_(logger) {}

bool SDLWindow::init(const char* title, int width, int height, Uint32 flags) {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        RCLCPP_ERROR(logger_, "SDL initialization failed: %s", SDL_GetError());
        return false;
    }

    window_ = SDL_CreateWindow(title, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, width, height, flags);
    if (!window_) {
        RCLCPP_ERROR(logger_, "Failed to create SDL window: %s", SDL_GetError());
        SDL_Quit();
        return false;
    }

    renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer_) {
        RCLCPP_ERROR(logger_, "Failed to create SDL renderer: %s", SDL_GetError());
        SDL_DestroyWindow(window_);
        SDL_Quit();
        return false;
    }

    return true;
}

SDLWindow::~SDLWindow() {
    if (renderer_) {
        SDL_DestroyRenderer(renderer_);
    }

    if (window_) {
        SDL_DestroyWindow(window_);
    }

    SDL_Quit();
}

void SDLWindow::render() {
    // Clear the screen (optional, depending on your use case)
    SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 255);
    SDL_RenderClear(renderer_);

    // Perform rendering here

    // Present the renderer
    SDL_RenderPresent(renderer_);
}
