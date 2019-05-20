#ifndef ROBOTPOSE_H
#define ROBOTPOSE_H

#include <cstdlib>
#include <iostream>
#include <math.h>
#include <position.h>


#include <vector>
#include <memory>

using namespace std;

namespace ecn
{
    class RobotPose: public Position
    {
        typedef std::unique_ptr<RobotPose> RobotPosePtr;

    public:

        float lin_vel; // linear velocity
        float ang_vel; // angular velocity
        // constructor from coordinates

        // constructor from base ecn::Point
        RobotPose(float _lin_vel, float _ang_vel, Position p ) : Position(p.x,p.y,p.theta) {
            lin_vel = _lin_vel;
            ang_vel = _ang_vel;
        }

        RobotPose(float _lin_vel, float _ang_vel, float _x, float _y, float _theta) : Position(_x, _y, _theta) {
            lin_vel = _lin_vel;
            ang_vel = _ang_vel;
        }

        // assignmnet operator
        void operator=(const RobotPose &p)
        {
            x = p.x;
            y = p.y;
            theta = p.theta;
            lin_vel = p.lin_vel;
            ang_vel = p.ang_vel;
        }

        // Suggest how the object of such data type should be printed
        friend std::ostream& operator<<(std::ostream& out, const RobotPose& p)
        {
            out << "(X:" << p.x << ",Y: " << p.y << ",theta:" << p.theta << ",lin_vel:" << p.lin_vel << ",ang_vel:" << p.ang_vel << ")" ;
            return out;
        }

        // TODO: how to say when two RobotPose are equal ?
        bool is(const RobotPose &other)
        {
            return x == other.x && y == other.y && theta == other.theta ;
        }

        // TODO: update the heuristic here
        double h(const RobotPose &goal, bool use_manhattan)
        {
            if(use_manhattan)
                return  abs(x-goal.x) + abs(y-goal.y);
            return 1.5*sqrt((x-goal.x)*(x-goal.x) + (y-goal.y)*(y-goal.y));
        }

        // TODO: how to implement this? 
        // Should probably resuse the isFree of the Position class here
        bool isFree(){
           }

        virtual void print(const RobotPose &parent);

        void start(); // Used inside the maze.h code 
        

        virtual void show(bool closed, const RobotPose &parent);

        static RobotPose begin();
        static RobotPose end();

        void test(); // This function was not being used anywhere 
        int distToParent();
        std::vector<RobotPosePtr> children();
        
    };
}

#endif // ROBOTPOSE_H
