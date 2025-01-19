# ifndef PP_CONTROLLER_H_
# define PP_CONTROLLER_H_
# include <vector>
# define M_PI 3.14159265358979323851
class pure_prusuit{
   public:

       struct point{
        float pos_x;
        float pos_y;
        float pos_z;
        float rot_x;
        float rot_y;
        float rot_z;

    };
    struct velocity{
        float linear_vel;
        float angular_vel;
    };

   pure_prusuit()=default;
   pure_prusuit(  float desired_speed,float des_tolerance_distance_to_goal,float des_lookahead_distance);
   void set_robot_pose(point current_pose);
   velocity compute_velocity();
   void set_path(std::vector<point> input_path);
   void set_goal(point input_goal);

   private:
   std::vector<point> path;
   point robot_pose{0.0,0.0,0.0,0.0,0.0,0.0};
   point goal;
   float desired_speed=0.2;
   float tolerance_distance_to_goal =0.1;
   float lookahead_distance=0.3;
   double min_angle_tolerance = 0.3;
   point get_lookahead_point();
   bool goal_checker();

};

# endif