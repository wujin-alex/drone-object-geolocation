cmake_minimum_required(VERSION 3.5)

find_package(DroneObjlocation REQUIRED)
include_directories(
    ${DroneObjlocation_INCLUDE_DIRS}
    "/usr/include/eigen3"
)
add_executable(demo_geolocation demo_geolocation.cpp)
target_link_libraries(demo_geolocation ${DroneObjlocation_LIBS})