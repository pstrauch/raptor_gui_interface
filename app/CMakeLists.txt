add_executable(raptor_gui_interface
src/main.cpp
)

target_include_directories(raptor_gui_interface PUBLIC include)

target_link_libraries(raptor_gui_interface PRIVATE
comm_lib
)

# Third party dependencies
target_link_libraries(raptor_gui_interface PRIVATE
MAVSDK::mavsdk_action
MAVSDK::mavsdk_offboard
MAVSDK::mavsdk_telemetry
MAVSDK::mavsdk_mocap
MAVSDK::mavsdk
fastcdr 
fastrtps
)