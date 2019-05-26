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

    protected:
        // tolerances along x,y and theta to be ignored whiel some comparison
        static float x_tol; //cm
        static float y_tol;
        static float theta_tol; // degree
        static float scale_factor; // 1 pixel = 1 cm : minimum step of the robot
        // static Maze maze;

    public:
        typedef std::unique_ptr<Position> PositionPtr;

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
        bool is(const Position &other);

        // TODO: update the heuristic here
        // Current heuristic is for a (2,0) type robot
        // since we can freely change theta at any given (x,y), the heuristic should be dependant on (x,y) at any given time
        // double h(const Position &goal, const Position &starting); //, bool use_manhattan)


        // TODO: check the implementation
        bool isFree();

//        float distFromPosition(const Position &p);
//        static float convertDegreeToRadian(int _deg);

    };
}

#endif
