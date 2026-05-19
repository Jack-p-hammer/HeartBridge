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

bool set_screen_audio(Image image, Audio_Prompt audio_prompt) {
    set_screen_image(image);
    set_audio_prompt(audio_prompt);
    return true;
}

bool set_screen_image(Image image) {
    const std::string& path = image_path(image);
    SDL_Texture* texture = IMG_LoadTexture(g_renderer, path.c_str());
    if (!texture) {
        std::cerr << "IMG_LoadTexture Error: " << IMG_GetError() << std::endl;
        return false;
    }

    // Render the texture to the screen
    SDL_RenderClear(g_renderer);
    SDL_RenderCopy(g_renderer, texture, nullptr, nullptr);
    SDL_RenderPresent(g_renderer);

    // Clean up the texture after rendering
    SDL_DestroyTexture(texture);

    return true;
}

bool set_audio_prompt(Audio_Prompt audio_prompt) {
    return true;
}

// Lookup table that converts Image enum values to file paths for the corresponding images
const std::string& image_path(Image image) {
    static const std::unordered_map<Image, std::string> image_paths = {
        {Image::UNFOLD, "Images/unfoldBmp.jpg"},
        {Image::CUT_CLOTHES, "Images/cutClothingBmp.jpg"},
        {Image::ALIGNMENT, "Images/alignmentBmp.jpg"},
        {Image::ZEROING_PREP, "Images/zeroingPrepBmp.jpg"},
        {Image::ZEROING, "Images/zeroingPrepBmp.jpg"}, // TODO: Update to actual zeroing image once we have it
        {Image::COMPRESSION_PREP, "Images/compressionsConfirmBmp.jpg"},
        {Image::COMPRESSION, "Images/compressionsBmp.jpg"},
        {Image::PAUSE, "Images/pausedBmp.jpg"},
        {Image::ABORT, "Images/abortBmp.jpg"},
        {Image::KNEEL_FAILURE, "Images/kneelFailureBmp.jpg"}
    };

    auto it = image_paths.find(image);
    if (it == image_paths.end()) {
        // Error throw instead of returning error code, but should never be hit if all Image enum values are handled properly
        throw std::invalid_argument("Invalid Image enum value");
    }
        
    return it->second;
}

// Lookup table that converts Audio_Prompt enum values to file paths for the corresponding audio prompts
const std::string& audio_prompt_path(Audio_Prompt audio_prompt) {
    static const std::unordered_map<Audio_Prompt, std::string> audio_paths = {
        {Audio_Prompt::STARTUP, "Audio/StartUpWav.wav"},
        {Audio_Prompt::UNFOLD, "Audio/unfoldExposeWav.wav"},
        {Audio_Prompt::CUT_CLOTHES, "Audio/cutClothingWav.wav"},
        {Audio_Prompt::ALIGNMENT, "Audio/alignmentWav.wav"},
        {Audio_Prompt::ZEROING_PREP, "Audio/zeroingPrepWav.wav"},
        {Audio_Prompt::ZEROING, "Audio/zeroingWav.wav"},
        {Audio_Prompt::COMPRESSION_PREP, "Audio/compressionsPrepWav.wav"},
        {Audio_Prompt::COMPRESSION, "Audio/compressionsWav.wav"},
        {Audio_Prompt::PAUSE, ""}, // TODO: Update with actual pause audio prompt once we have it
        {Audio_Prompt::ABORT, ""},
        {Audio_Prompt::KNEEL_FAILURE, ""}
    };

    auto it = audio_paths.find(audio_prompt);
    if (it == audio_paths.end()) {
        // Error throw instead of returning error code, but should never be hit if all Audio_Prompt enum values are handled properly
        throw std::invalid_argument("Invalid Audio_Prompt enum value");
    }
        
    return it->second;
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