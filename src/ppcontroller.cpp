#include "pure_prusuit/ppcontroller.hpp"
#include <iostream>
#include<stdexcept> 
#include<cmath>
#include<algorithm>
#include <numbers>
double normalize_angle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

PurePrusuit::PurePrusuit(float des_vel,float des_tolerance_distance_to_goal ,float des_lookahead_distance)
                        :desired_speed_{des_vel},tolerance_distance_to_goal_{des_tolerance_distance_to_goal},
                        lookahead_distance_{des_lookahead_distance}{}

void PurePrusuit::setRobotPose(pose current_pose){
          robot_pose_=current_pose;
}
void PurePrusuit::setGoal(pose input_goal){
    this->goal_= input_goal;
}
bool PurePrusuit::goalChecker(){
    float distance_to_goal= std::sqrt(
        std::pow((goal_.position.x -robot_pose_.position.x),2)
        +std::pow((goal_.position.y - robot_pose_.position.y),2));
    if(distance_to_goal<=tolerance_distance_to_goal_){
        return true;
    }
    return false;
}
void PurePrusuit::setPath(std::vector<pose> input_path){
    this->path_= input_path;
}
PurePrusuit::pose PurePrusuit::getCarrotPoint(){
       std::remove_if(path_.begin(), path_.end(),
        [&](const auto & ps) {
            double dx = ps.position.x - robot_pose_.position.x;
            double dy = ps.position.y - robot_pose_.position.y;
            double heading_x = std::cos(robot_pose_.orientation.z);
            double heading_y = std::sin(robot_pose_.orientation.z);
            return (dx * heading_x + dy * heading_y) < 0;  
        });
       
    auto pose_it=std::find_if(path_.begin(),path_.end(),
                        [&](const auto & ps){
                            return  std::sqrt(std::pow((ps.position.x -robot_pose_.position.x),2)
                                    +std::pow((ps.position.y - robot_pose_.position.y),2)) >=lookahead_distance_;});
    if(pose_it == path_.end()){
        pose_it = std::prev(path_.end());
    }
    return *pose_it;
}


PurePrusuit::velocity  PurePrusuit::computeVelocityCommands( const pose & robot_pose,  double current_speed) {
    
    if (path_.empty()) {
        throw std::runtime_error("path is empty");
    }
    setRobotPose(robot_pose);
    if (goalChecker()) {
        std::cout << "Robot has arrived at the goal." << std::endl;
        return velocity{0.0, 0.0};  
    }
    
    auto carrot_point = getCarrotPoint();
    const double carrot_distance2 = std::pow((carrot_point.position.x - robot_pose_.position.x), 2) +
                                     std::pow((carrot_point.position.y - robot_pose_.position.y), 2);
    std::cout << "Carrot Point: (" << carrot_point.position.x << ", " << carrot_point.position.y << ")" << std::endl;

    double curvature = 0.0;
    if (carrot_distance2 > 0.001) {
        curvature = 2 * robot_pose_.position.y / carrot_distance2;
    }

    velocity output_vel;
    double angle_to_path = atan2(carrot_point.position.y - robot_pose_.position.y,
                                 carrot_point.position.x - robot_pose_.position.x);
    double yaw_error = normalize_angle(angle_to_path - robot_pose_.orientation.z); 

    if (std::abs(yaw_error) > min_angle_tolerance_) {
        output_vel.linear_vel = 0.0;
        output_vel.angular_vel = 0.5 * std::copysign(1.0, yaw_error); 
        std::cout << "Rotating to path with angular velocity: " << output_vel.angular_vel << std::endl;
        return output_vel; 
    }

 
    output_vel.linear_vel = desired_speed_;
    output_vel.angular_vel = curvature * output_vel.linear_vel;
    std::cout << "Current linear velocity: " << output_vel.linear_vel
              << " | Current angular velocity: " << output_vel.angular_vel << std::endl;

        return output_vel;  
}

void PurePrusuit::setSpeedLimits(const double & speed_limit, const bool & percentage){
    if(!percentage){
        desired_speed_=speed_limit;
    }
    desired_speed_=desired_speed_*(speed_limit/100);

}