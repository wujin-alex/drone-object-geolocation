#include <stdlib.h>
#include <stdio.h>

#include <iostream>
#include <vector>
#include "geolocation/drone_objlocation.h"

using namespace std;

DroneObjlocation geo_location = DroneObjlocation();

void test01()
{
    std::map<std::string, std::vector<float>> result;

    std::vector<int> uv = {960, 540};
    float focal = 4.3;
    float distance = 100;
    int   distance_type = 1;

    std::vector<float> euler_camera   = {0.0, 0.0, 0.0};
    std::vector<float> euler_drone    = {0.0, 0.0, 0.0};
    std::vector<float> position_drone = {30.0, 104.0, 100.0, 300.0};

    geo_location.set_parameter((uint16_t)1920, (uint16_t)1080, 4.3, 1482.0, 1482.0, 960.0, 540.0);
    result = geo_location.get_target_location(uv, focal, distance, euler_camera, euler_drone, position_drone, distance_type); 

    printf("p_b: %f, %f, %f\n", result["p_b"][0], result["p_b"][1], result["p_b"][2]);
    
}
int main(int argc, char *argv[])
{
    test01();
    return 0;
}