#include<gtest/gtest.h>
#include "../include/pure_prusuit/ppcontroller.hpp"
#include<chrono>
#include<thread>
#include<cmath>
class pp_controller: public pure_prusuit ,public testing::Test{
protected:
 void SetUp()override{
    set_goal({5.0, 5.0, 0.0, 0.0, 0.0, 0.0});

    // Define the path
    auto path = std::vector<pure_prusuit::pose>{
        {1, 1, 0, 0, 0, 0},
        {2, 2, 0, 0, 0, 0},
        {3, 3, 0, 0, 0, 0},
        {4, 4, 0, 0, 0, 0},
        {5, 5, 0, 0, 0, 0}};
    setPath(path);
 }
//pure_prusuit pp_localplanner;
};

TEST_F(pp_controller,setPath){

   EXPECT_EQ(5,path.size());
}

TEST_F(pp_controller,setgoal){
    ASSERT_EQ(5.0f,goal.position.x);
    ASSERT_EQ(5.0f,goal.position.y);
    ASSERT_EQ(0.0f,goal.position.z);
    ASSERT_EQ(0.0f,goal.orientation.x);
    ASSERT_EQ(0.0f,goal.orientation.y);
    ASSERT_EQ(0.0f,goal.orientation.z);

}

TEST_F(pp_controller,setRobotPose){
    set_robot_pose({5,5,0,0,0,0.7});
    ASSERT_EQ(5.0f,robot_pose.position.x);
    ASSERT_EQ(5.0f,robot_pose.position.y);
    ASSERT_EQ(0.0f,robot_pose.position.z);
    ASSERT_EQ(0.0f,robot_pose.orientation.x);
    ASSERT_EQ(0.0f,robot_pose.orientation.y);
    ASSERT_EQ(0.7f,robot_pose.orientation.z);
}
TEST_F(pp_controller,speedlimit){
    desired_speed=1.0;
    setSpeedLimits(90,true);
    EXPECT_EQ(0.9f,desired_speed);
}
TEST_F(pp_controller,goalcheck){
     set_goal({8.0, 5.0, 0.0, 0.0, 0.0, 0.0});
     set_robot_pose({7.94, 5.05, 0.0, 0.0, 0.0, 0.0});
     EXPECT_EQ(true,goal_checker());
}

TEST_F(pp_controller,getcarrotpoint){
   set_robot_pose({2.5, 2.2, 0.0, 0.0, 0.0, 0.0});
   auto pose =get_lookahead_point();
    ASSERT_EQ(3.0f,pose.position.x);
    ASSERT_EQ(3.0f,pose.position.y);
    ASSERT_EQ(0.0f,pose.position.z);
    ASSERT_EQ(0.0f,pose.orientation.x);
    ASSERT_EQ(0.0f,pose.orientation.y);
    ASSERT_EQ(0.0f,pose.orientation.z);  
}


 TEST_F(pp_controller,computeVelocity){
   
    int sim_steps = 900;
    pure_prusuit::pose current_state = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::chrono::milliseconds sim_step(100);

    for (int i = 0; i <= sim_steps; i++) {

        pure_prusuit::velocity current_vel = computeVelocityCommands(current_state,0.0);

        current_state.position.x += current_vel.linear_vel * std::cos(current_state.orientation.z) * 0.1;
        current_state.position.y += current_vel.linear_vel * std::sin(current_state.orientation.z) * 0.1;
        current_state.orientation.z += current_vel.angular_vel * 0.1;

        current_state.orientation.z = std::atan2(std::sin(current_state.orientation.z), std::cos(current_state.orientation.z));


        std::this_thread::sleep_for(sim_step);
    }
    EXPECT_TRUE(goal_checker());
}


