#ifndef POSITION_H
#define POSITION_H

#include <cstdlib>
#include <iostream>
#include <math.h>
#include <point.h>


#include <vector>
#include <memory>

#include <maze.h>

//using namespace std;

namespace ecn
{
    class Position
    {
        // TODO:
        // 1. probably add a POint(x,y) as a internal variable or inherit from it ?
        // 2. Add a begin and end poisition as static variables so that all the nodes know about it!
        // Or do we want to do that for just the RobotPose ?
        typedef std::unique_ptr<Position> PositionPtr;

    protected:
        // tolerances along x,y and theta to be ignored whiel some comparison
        static float x_tol; //cm
        static float y_tol;
        static float theta_tol; // degree
        static float scale_factor; // 1 pixel = 1 cm : minimum step of the robot
        // static Maze maze;
    public:
        float x;
        float y;
        float theta;

        // constructor from coordinates
        Position(float _x, float _y, float _theta){
            x = _x;
            y = _y;
            theta = _theta;
        }

        // constructor from base ecn::Point
        Position(Point p, float _theta){
            x = p.x;
            y = p.y;
            theta = _theta;
        }


        // assignmnet operator
        void operator=(const Position &p)
        {
            x = p.x;
            y = p.y;
            theta = p.theta;
        }


        friend std::ostream& operator<<(std::ostream& out, const Position& p)
        {
            out << "(X:" << p.x << ",Y: " << p.y << ",theta:" << p.theta << ")" ;
            return out;
        }

        // TODO: how to say when two positions are equal - added tolerances along x, y and theta
        bool is(const Position &other)
        {
            //return x == other.x && y == other.y && theta == other.theta ;
//            float o_x = other.x;
//            float o_y = other.y;
//            float o_theta = other.theta;

            if((other.x - x_tol <= x <= other.x + x) && (other.y - y_tol <= y <= other.y + y) && (other.theta - theta_tol <= theta <= other.theta + theta_tol)){
                return 1;
            }
            return 0;
        }

        // TODO: update the heuristic here
        // Current heuristic is for a (2,0) type robot
        // since we can freely change theta at any given (x,y), the heuristic should be dependant on (x,y) at any given time
        double h(const Position &goal, const Position &starting) //, bool use_manhattan)
        {
            // if(use_manhattan)
            //     return  abs(x-goal.x) + abs(y-goal.y);
            // return 1.5*sqrt((x-goal.x)*(x-goal.x) + (y-goal.y)*(y-goal.y));
            float x_dist = (x - starting.x)/(goal.x - starting.x);
            float y_dist = (y - starting.y)/(goal.y - starting.y);
            float theta_dist = (theta - starting.theta)/(goal.theta - starting.x);
            return sqrt(pow(x_dist,2) + pow(y_dist,2) + pow(theta_dist,2));

        }

        // TODO: how to implement this? 
        // 1. Inherit Point and use maze
        // 2. Don't worry about this and assign this as the responsibility of the user ?
        bool isFree(){
            int point_x = (int) floor(x/scale_factor);
            int point_y = (int) floor(y/scale_factor);
            Point temp_p(point_x,point_y);
            return temp_p.isFree();
            }

        // virtual void print(const Position &parent);

        //// void start(); // Used inside the maze.h code
        

        //// virtual void show(bool closed, const Position &parent);

        // static Position begin();
        // static Position end();

        // void test(); // This function was not being used anywhere
        // int distToParent();
        // std::vector<PositionPtr> children();
        

    };
}

#endif
