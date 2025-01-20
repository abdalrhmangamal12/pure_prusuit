#ifndef CONTROLLER_HH_
#define CONTROLLER_HH_

#include <vector>
class controller{
    public:
    struct Position {
        float x;
        float y;
        float z;
    };

    struct Orientation {
        float x;
        float y;
        float z;
    };

    struct pose {
         Position position;
         Orientation orientation;
    };

    struct velocity{
        float linear_vel;
        float angular_vel;
    };
    virtual  void setPath(std::vector<pose> input_path)=0;
    virtual  void setSpeedLimits(const double & speed_limit, const bool & percentage)=0;
    virtual  velocity computeVelocityCommands( const pose & robot_pose, double current_speed )=0;

};
#endif