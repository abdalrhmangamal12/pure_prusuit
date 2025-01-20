#include "pure_prusuit/ppcontroller.hpp"
#include <iostream>
#include<stdexcept> 
#include<cmath>
#include<algorithm>
#include <numbers>
double normalize_angle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

pure_prusuit::pure_prusuit(float des_vel,float des_tolerance_distance_to_goal ,float des_lookahead_distance)
                        :desired_speed{des_vel},tolerance_distance_to_goal{des_tolerance_distance_to_goal},
                        lookahead_distance{des_lookahead_distance}{}

void pure_prusuit::set_robot_pose(pose current_pose){
          robot_pose=current_pose;
}
void pure_prusuit::set_goal(pose input_goal){
    this->goal = input_goal;
}
bool pure_prusuit::goal_checker(){
    float distance_to_goal= std::sqrt(
        std::pow((goal.position.x -robot_pose.position.x),2)
        +std::pow((goal.position.y - robot_pose.position.y),2));
    if(distance_to_goal<=tolerance_distance_to_goal){
        return true;
    }
    return false;
}
void pure_prusuit::setPath(std::vector<pose> input_path){
    this->path= input_path;
}
pure_prusuit::pose pure_prusuit::get_lookahead_point(){
       std::remove_if(path.begin(), path.end(),
        [&](const auto & ps) {
            double dx = ps.position.x - robot_pose.position.x;
            double dy = ps.position.y - robot_pose.position.y;
            double heading_x = std::cos(robot_pose.orientation.z);
            double heading_y = std::sin(robot_pose.orientation.z);
            return (dx * heading_x + dy * heading_y) < 0;  
        });
       
    auto pose_it=std::find_if(path.begin(),path.end(),
                        [&](const auto & ps){
                            return  std::sqrt(std::pow((ps.position.x -robot_pose.position.x),2)
                                    +std::pow((ps.position.y - robot_pose.position.y),2)) >=lookahead_distance;});
    if(pose_it == path.end()){
        pose_it = std::prev(path.end());
    }
    return *pose_it;
}


pure_prusuit::velocity  pure_prusuit::computeVelocityCommands( const pose & robot_pose,  double current_speed) {
    
    if (path.empty()) {
        throw std::runtime_error("path is empty");
    }
    set_robot_pose(robot_pose);
    if (goal_checker()) {
        std::cout << "Robot has arrived at the goal." << std::endl;
        return velocity{0.0, 0.0};  
    }
    
    auto carrot_point = get_lookahead_point();
    const double carrot_distance2 = std::pow((carrot_point.position.x - robot_pose.position.x), 2) +
                                     std::pow((carrot_point.position.y - robot_pose.position.y), 2);
    std::cout << "Carrot Point: (" << carrot_point.position.x << ", " << carrot_point.position.y << ")" << std::endl;

    double curvature = 0.0;
    if (carrot_distance2 > 0.001) {
        curvature = 2 * robot_pose.position.y / carrot_distance2;
    }

    velocity output_vel;
    double angle_to_path = atan2(carrot_point.position.y - robot_pose.position.y,
                                 carrot_point.position.x - robot_pose.position.x);
    double yaw_error = normalize_angle(angle_to_path - robot_pose.orientation.z); 

    if (std::abs(yaw_error) > min_angle_tolerance) {
        output_vel.linear_vel = 0.0;
        output_vel.angular_vel = 0.5 * std::copysign(1.0, yaw_error); 
        std::cout << "Rotating to path with angular velocity: " << output_vel.angular_vel << std::endl;
        return output_vel; 
    }

 
    output_vel.linear_vel = desired_speed;
    output_vel.angular_vel = curvature * output_vel.linear_vel;
    std::cout << "Current linear velocity: " << output_vel.linear_vel
              << " | Current angular velocity: " << output_vel.angular_vel << std::endl;

        return output_vel;  
}

void pure_prusuit::setSpeedLimits(const double & speed_limit, const bool & percentage){
    if(percentage){
        desired_speed=desired_speed*(speed_limit/100);
    }
    else{
        desired_speed = speed_limit;
    }
}