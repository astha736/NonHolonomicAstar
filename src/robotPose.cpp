#include <robotPose.h>

namespace ecn
{

// int RobotPose::distToParent()
//     {
//         // in cell-based motion, the distance to the parent is always 1
//         return 1;
//     }


std::vector<RobotPose::RobotPosePtr> RobotPose::children()
    {
        // this method should return  all positions reachable from this one
        std::vector<RobotPose::RobotPosePtr> generated;

        return generated;
    }


} // END of namespace