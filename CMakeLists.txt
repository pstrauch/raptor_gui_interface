cmake_minimum_required(VERSION 3.16)
project(raptor_gui_interface VERSION 0.1)
set(CMAKE_CXX_STANDARD 17)

# Find requirements
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
find_package(MAVSDK REQUIRED)


add_subdirectory(lib)
add_subdirectory(external)
add_subdirectory(app)
add_subdirectory(test)