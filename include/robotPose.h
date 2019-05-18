#ifndef ROBOTPOSE_H
#define ROBOTPOSE_H

#include <cstdlib>
#include <iostream>
#include <math.h>
#include <position.h>


#include <vector>
#include <memory>

//using namespace std;

namespace ecn
{
    class RobotPose: public Position
    {
        typedef std::unique_ptr<RobotPose> RobotPosePtr;

    public:

        int lin_vel; // linear velocity
        int ang_vel; // angular velocity

        // constructor from coordinates

        // constructor from base ecn::Point
        RobotPose(int _lin_vel, int _ang_vel, Position p ) : Position(p.x,p.y,p.theta) {
            lin_vel = _lin_vel;
            ang_vel = _ang_vel;
        }

        RobotPose(int _lin_vel, int _ang_vel, int _x, int _y, int _theta) : Position(_x, _y, _theta) {
            lin_vel = _lin_vel;
            ang_vel = _ang_vel;
        }


        //int distToParent();

        std::vector<RobotPosePtr> children();
        
    };
}

#endif // ROBOTPOSE_H
