#include <robotPose.h>

namespace ecn
{

// TODO: write this print correctly
void RobotPose::print(const RobotPose &parent)
{
}

// used in the maze function, have to check and update 
// TODO: write this print correctly
void RobotPose::start()
{
}

// online print, color depends on closed / open set
// TODO: write this print correctly
void RobotPose::show(bool closed, const RobotPose & parent)
{
}

// TODO: these should set some static variables 
RobotPose RobotPose::begin(){
}

// TODO: these should set some static variables 
RobotPose RobotPose::end()
{
}

// TODO: write this print correctly
int RobotPose::distToParent()
    {
        // in cell-based motion, the distance to the parent is always 1
        return 1;
    }

// TODO: write this print correctly
std::vector<RobotPose::RobotPosePtr> RobotPose::children()
    {
        // this method should return  all positions reachable from this one
        std::vector<RobotPose::RobotPosePtr> generated;

        return generated;
    }


} // END of namespace