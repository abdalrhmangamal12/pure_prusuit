#include "../include/ppcontroller.hpp"
#include<vector>
#include<cmath>
#include<chrono>
#include<thread>
#include<iostream>
int main(){
    pure_prusuit pp_localplanner;
    pp_localplanner.set_robot_pose({0.0,0.0,0.0,0.0,0.0,0.0});
    pp_localplanner.set_goal({5.0,5.0,0.0,0.0,0.0,0.0});
    auto path=std::vector<pure_prusuit::point>{{1,1,0,0,0,0},
                                          {2,2,0,0,0,0},
                                          {3,3,0,0,0,0},
                                          {4,4,0,0,0,0},
                                          {5,5,0,0,0,0}};
    pp_localplanner.set_path(path);
    int sim_steps = 1000;
    pure_prusuit::point current_state;
     std::chrono::milliseconds sim_step(100);
    for(int i =0 ; i<=sim_steps;i++){
      
    pure_prusuit::velocity current_vel=pp_localplanner.compute_velocity();
    current_state.pos_x+=current_vel.linear_vel*std::cos(current_state.rot_z)*0.1;
    current_state.pos_y+=current_vel.linear_vel*std::sin(current_state.rot_z)*0.1;
    current_state.rot_z+=current_vel.angular_vel*0.1;
    std::cout<<"current x :"<<current_state.pos_x<<"current y is :"<<current_state.pos_y<<std::endl;
    pp_localplanner.set_robot_pose({current_state.pos_x,current_state.pos_y,0.0,0.0,0.0,current_state.rot_z});
   
    
     std::this_thread::sleep_for(sim_step);
    }
    return 0;
}