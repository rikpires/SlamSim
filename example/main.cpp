#include <iostream>
#include <string>
#include "converter.h"
#include "system_simulator.h"
#include "viewer.h"

int main()
{
    std::string trajectory_file_path = "../cfg/trajectory.ini";
    slam_sim::Simulator simulator ( trajectory_file_path );
    simulator.Run();
    // std::cin.get();
}