# ifndef PP_CONTROLLER_H_
# define PP_CONTROLLER_H_
#include "../controller.hpp"
# include <vector>
# define M_PI 3.14159265358979323851
class pure_prusuit: public controller{
   public:
   pure_prusuit()=default;
   pure_prusuit(  float desired_speed,float des_tolerance_distance_to_goal,float des_lookahead_distance);
   void set_robot_pose(pose current_pose);

   void setSpeedLimits(const double & speed_limit, const bool & percentage)override;
   velocity computeVelocityCommands( const pose & robot_pose, double current_speed)override;
   void setPath(std::vector<pose> input_path)override;

   void set_goal(pose input_goal);

   private:
   std::vector<pose> path;
   pose robot_pose{{0.0,0.0,0.0},
                     {0.0,0.0,0.0}};
   pose goal;
   float desired_speed=1.2;
  
   float tolerance_distance_to_goal =0.1;
   float lookahead_distance=0.5;
   double min_angle_tolerance = 0.5;
   pose get_lookahead_point();
   bool goal_checker();

};

# endif