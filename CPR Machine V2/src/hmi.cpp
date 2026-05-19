/**
 * @file hmi.cpp
 * @brief Implementation of Human-Machine Interface functions for the CPR machine.
 *
 * This source file contains the implementations of human-machine interface operations
 * in the CPR machine.
 *
 * @date 2026-05-10
 * @version 1.0
 */

#include "hmi.hpp"

// Internal state — not exposed in the header
namespace {
    SDL_Window*   g_window   = nullptr;
    SDL_Renderer* g_renderer = nullptr;
}    

Error_Code init_screens() {
    // Initialize SDL video and audio subsystems
    if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) != 0) {
        std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return Error_Code::ERROR_INIT_FAILURE;
    }

    // Initialize SDL_image for JPG support
    int img_flags = IMG_INIT_JPG;
    if((IMG_Init(img_flags) & img_flags) != img_flags) {
        std::cerr << "IMG_Init Error: " << IMG_GetError() << std::endl;
        return Error_Code::ERROR_INIT_FAILURE;
    }
    
    // Get current display mode to set window size to full screen
    SDL_DisplayMode mode;
    if(SDL_GetCurrentDisplayMode(0, &mode) < 0) {
        std::cerr << "SDL_GetCurrentDisplayMode Error: " << SDL_GetError() << std::endl;
        return Error_Code::ERROR_INIT_FAILURE;
    }

    // Create a full-screen window and renderer
    g_window = SDL_CreateWindow(
        "CPR Machine", 
        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 
        mode.w, mode.h, 
        SDL_WINDOW_FULLSCREEN_DESKTOP | SDL_WINDOW_SHOWN
    );
    if(!g_window) {
        std::cerr << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
        return Error_Code::ERROR_INIT_FAILURE;
    }

    // Create renderer with hardware acceleration and vsync
    g_renderer = SDL_CreateRenderer(
        g_window, -1, 
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC
    );
    if(!g_renderer) {
        std::cerr << "SDL_CreateRenderer Error: " << SDL_GetError() << std::endl;
        return Error_Code::ERROR_INIT_FAILURE;
    }

    // Initialize SDL_mixer for audio playback
    // 44100 Hz sample rate, default format, mono audio (1), 2048 byte chunks
    if(Mix_OpenAudio(44100, MIX_DEFAULT_FORMAT, 1, 2048) < 0) {
        std::cerr << "Mix_OpenAudio Error: " << Mix_GetError() << std::endl;
        return Error_Code::ERROR_INIT_FAILURE;
    }

    // If we got this far, initialization successful
    return Error_Code::NORMAL_OPERATION;
}

bool set_screen_audio() {
    return true;
}

void enable_lasers() {
    return;
}

void disable_lasers() {
    return;
}

Error_Code enable_next_button() {
    return Error_Code::NORMAL_OPERATION;
}

Error_Code disable_next_button() {
    return Error_Code::NORMAL_OPERATION;
}

Error_Code enable_pause_button() {
    return Error_Code::NORMAL_OPERATION;
}

Error_Code disable_pause_button() {
    return Error_Code::NORMAL_OPERATION;
}

Error_Code read_buttons() {
    return Error_Code::NORMAL_OPERATION;
}