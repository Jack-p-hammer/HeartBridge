cmake_minimum_required(VERSION 3.10)

project(CPR_Machine_V2)

set(CMAKE_CXX_STANDARD 17)

# Use vcpkg if on Windows
if(WIN32)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake")
endif()

file(GLOB_RECURSE SOURCES "src/*.cpp")

add_executable(CPR_Machine_V2 ${SOURCES})

target_include_directories(CPR_Machine_V2 PRIVATE src/)

find_package(SDL2 REQUIRED)
find_package(SDL2_image REQUIRED)
find_package(SDL2_mixer REQUIRED)

target_link_libraries(CPR_Machine_V2 PRIVATE 
    SDL2::SDL2
    SDL2::SDL2main
    SDL2_image::SDL2_image
    SDL2_mixer::SDL2_mixer
)