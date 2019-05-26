#include <position.h>

namespace ecn
{

float Position::x_tol = 0; //cm
float Position::y_tol = 0;
float Position::theta_tol = 0; // degree
float Position::scale_factor = 1; // 1 pixel = 1 cm : minimum step of the robot

// Two positions are equal
bool Position::is(const Position &other)
{
    if((other.x - x_tol <= x <= other.x + x) &&
       (other.y - y_tol <= y <= other.y + y) &&
       (other.theta - theta_tol <= theta <= other.theta + theta_tol)){
        return 1;
    }
    return 0;
}

// This heurstic is currently used by the RobotPose class as well
double Position::h(const Position &goal, const Position &starting) //, bool use_manhattan)
{
    // if(use_manhattan)
    //     return  abs(x-goal.x) + abs(y-goal.y);
    // return 1.5*sqrt((x-goal.x)*(x-goal.x) + (y-goal.y)*(y-goal.y));
    float x_dist = (x - starting.x)/(goal.x - starting.x);
    float y_dist = (y - starting.y)/(goal.y - starting.y);
    float theta_dist = (theta - starting.theta)/(goal.theta - starting.x);
    return sqrt(pow(x_dist,2) + pow(y_dist,2) + pow(theta_dist,2));

}

// This isFree is currently used by the RobotPose class as well
// TODO: is this much okay??
bool Position::isFree(){
    if(Point::maze.isFree(x, y)){
        return true;
    }
    return false;

    }

// TODO: Put some values here
//float Position::distFromPosition(const Position &p){
//    //
//    return 1;
//}

}
