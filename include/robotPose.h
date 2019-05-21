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
        //MAYBE MAKE ALL INT

        float r_vel; // right angular velocity
        float l_vel; // left angular velocity

        static float vel_max; // max angular velocity for any wheel
        static float vel_min; // min angular velocity for any wheel
        static float vel_tol; // angular velocity tolerance to be used in comparison
        static float vel_parent_tol; // range in which child velocities should lie wrt parent

        // constructor from coordinates

        // constructor from base ecn::Point
        RobotPose(float _r_vel, float _l_vel, Position p ) : Position(p.x,p.y,p.theta) {
            r_vel = _r_vel;
            l_vel = _l_vel;
        }

        RobotPose(float _r_vel, float _l_vel, float _x, float _y, float _theta) : Position(_x, _y, _theta) {
            r_vel = _r_vel;
            l_vel = _l_vel;
        }

        // assignmnet operator
        void operator=(const RobotPose &p)
        {
            x = p.x;
            y = p.y;
            theta = p.theta;
            r_vel = p.r_vel;
            l_vel = p.l_vel;
        }

        // Suggest how the object of such data type should be printed
        friend std::ostream& operator<<(std::ostream& out, const RobotPose& p)
        {
            out << "(X:" << p.x << ",Y: " << p.y << ",theta:" << p.theta << ",r_vel:" << p.r_vel << ",l_vel:" << p.l_vel << ")" ;
            return out;
        }
        bool is(const RobotPose &other)
        {
            if((other.x - x_tol <= x <= other.x + x) && (other.y - y_tol <= y <= other.y + y) && (other.theta - theta_tol <= theta <= other.theta + theta_tol) && (other.l_vel - vel_tol <= l_vel <= other.l_vel + vel_tol) && (other.r_vel - vel_tol <= r_vel <= other.r_vel + vel_tol))
            {
                return 1;
            }
            return 0;
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
