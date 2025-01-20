#include "pure_prusuit/ppcontroller.hpp"
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include <iostream>

int main() {

  
    PurePrusuit pp_localplanner;
    pp_localplanner.setGoal({5.0, 5.0, 0.0, 0.0, 0.0, 0.0});


    controller &local_planner=pp_localplanner;
    // Define the path

    auto path = std::vector<controller::pose>{

        {1, 1, 0, 0, 0, 0},
        {2, 2, 0, 0, 0, 0},
        {3, 3, 0, 0, 0, 0},
        {4, 4, 0, 0, 0, 0},
        {5, 5, 0, 0, 0, 0}};

    local_planner.setPath(path);
    int sim_steps = 2000;
    controller::pose current_state = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    std::chrono::milliseconds sim_step(100);

    for (int i = 0; i <= sim_steps; i++) {


        auto current_vel = local_planner.computeVelocityCommands(current_state,0.0);

        current_state.position.x += current_vel.linear_vel * std::cos(current_state.orientation.z) * 0.1;
        current_state.position.y += current_vel.linear_vel * std::sin(current_state.orientation.z) * 0.1;
        current_state.orientation.z += current_vel.angular_vel * 0.1;

        current_state.orientation.z = std::atan2(std::sin(current_state.orientation.z), std::cos(current_state.orientation.z));

        std::cout << "Step: " << i
                  << " | Current X: " << current_state.position.x
                  << " | Current Y: " << current_state.position.y
                  << " | Heading (rot_z): " << current_state.orientation.z << std::endl;

      


        std::this_thread::sleep_for(sim_step);
    }

    return 0;
}
