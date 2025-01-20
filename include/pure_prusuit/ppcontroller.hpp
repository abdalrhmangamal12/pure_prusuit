# ifndef PP_CONTROLLER_H_
# define PP_CONTROLLER_H_
#include "../controller.hpp"
# include <vector>
# define M_PI 3.14159265358979323851

class PurePrusuit: public controller{
   public:
   PurePrusuit()=default;
   PurePrusuit(  float desired_speed,float des_tolerance_distance_to_goal,float des_lookahead_distance);
   // this pure prusuit class is not movable and not copyable
   PurePrusuit(const PurePrusuit & other)=delete;
   PurePrusuit( PurePrusuit && other)=delete;
   PurePrusuit & operator=(const PurePrusuit & other)=delete;
   PurePrusuit & operator=( PurePrusuit && other)=delete;

   void setRobotPose(pose current_pose);
   void setSpeedLimits(const double & speed_limit, const bool & percentage)override;
   velocity computeVelocityCommands( const pose & robot_pose, double current_speed)override;
   void setPath(std::vector<pose> input_path)override;

   void setGoal(pose input_goal);

   protected:
   std::vector<pose> path_;
   pose robot_pose_{{0.0,0.0,0.0},
                     {0.0,0.0,0.0}};
   pose goal_;
   float desired_speed_=1.2;
  
   float tolerance_distance_to_goal_ =0.1;
   float lookahead_distance_=0.5;
   double min_angle_tolerance_ = 0.5;
   pose getCarrotPoint();
   bool goalChecker();

};

# endif