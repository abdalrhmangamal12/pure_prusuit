#include<gtest/gtest.h>
#include "../include/pure_prusuit/ppcontroller.hpp"
#include<chrono>
#include<thread>
#include<cmath>
class pp_controller: public PurePrusuit ,public testing::Test{
protected:
 void SetUp()override{
    setGoal({5.0, 5.0, 0.0, 0.0, 0.0, 0.0});

    // Define the path
    auto path = std::vector<controller::pose>{
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

   EXPECT_EQ(5,path_.size());
}

TEST_F(pp_controller,setgoal){
    ASSERT_EQ(5.0f,goal_.position.x);
    ASSERT_EQ(5.0f,goal_.position.y);
    ASSERT_EQ(0.0f,goal_.position.z);
    ASSERT_EQ(0.0f,goal_.orientation.x);
    ASSERT_EQ(0.0f,goal_.orientation.y);
    ASSERT_EQ(0.0f,goal_.orientation.z);

}

TEST_F(pp_controller,setRobotPose){
    setRobotPose({5,5,0,0,0,0.7});
    ASSERT_EQ(5.0f,robot_pose_.position.x);
    ASSERT_EQ(5.0f,robot_pose_.position.y);
    ASSERT_EQ(0.0f,robot_pose_.position.z);
    ASSERT_EQ(0.0f,robot_pose_.orientation.x);
    ASSERT_EQ(0.0f,robot_pose_.orientation.y);
    ASSERT_EQ(0.7f,robot_pose_.orientation.z);
}
TEST_F(pp_controller,speedlimit){
    desired_speed_=1.0;
    setSpeedLimits(90,true);
    EXPECT_EQ(0.9f,desired_speed_);
}
TEST_F(pp_controller,goalcheck){
     setGoal({8.0, 5.0, 0.0, 0.0, 0.0, 0.0});
     setRobotPose({7.94, 5.05, 0.0, 0.0, 0.0, 0.0});
     EXPECT_TRUE(goalChecker());
}

TEST_F(pp_controller,getcarrotpoint){
   setRobotPose({2.5, 2.2, 0.0, 0.0, 0.0, 0.0});
   auto pose =getCarrotPoint();
    ASSERT_EQ(3.0f,pose.position.x);
    ASSERT_EQ(3.0f,pose.position.y);
    ASSERT_EQ(0.0f,pose.position.z);
    ASSERT_EQ(0.0f,pose.orientation.x);
    ASSERT_EQ(0.0f,pose.orientation.y);
    ASSERT_EQ(0.0f,pose.orientation.z);  
}


 TEST_F(pp_controller,computeVelocity){
   
    int sim_steps = 900;
    controller::pose current_state = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::chrono::milliseconds sim_step(100);

    for (int i = 0; i <= sim_steps; i++) {

        controller::velocity current_vel = computeVelocityCommands(current_state,0.0);

        current_state.position.x += current_vel.linear_vel * std::cos(current_state.orientation.z) * 0.1;
        current_state.position.y += current_vel.linear_vel * std::sin(current_state.orientation.z) * 0.1;
        current_state.orientation.z += current_vel.angular_vel * 0.1;

        current_state.orientation.z = std::atan2(std::sin(current_state.orientation.z), std::cos(current_state.orientation.z));


        std::this_thread::sleep_for(sim_step);
    }
    EXPECT_TRUE(goalChecker());
}


