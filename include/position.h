#ifndef POSITION_H
#define POSITION_H

#include <cstdlib>
#include <iostream>
#include <math.h>
#include <point.h>


#include <vector>
#include <memory>

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

        // TODO: how to say when two positions are equal ?
        bool is(const Position &other)
        {
            return x == other.x && y == other.y && theta == other.theta ;
        }

        // TODO: update the heuristic here
        double h(const Position &goal, bool use_manhattan)
        {
            if(use_manhattan)
                return  abs(x-goal.x) + abs(y-goal.y);
            return 1.5*sqrt((x-goal.x)*(x-goal.x) + (y-goal.y)*(y-goal.y));
        }

        // TODO: how to implement this? 
        // 1. Inherit Point and use maze
        // 2. Don't worry about this and assign this as the responsibility of the user ?
        bool isFree(){
           }

        virtual void print(const Position &parent);

        void start(); // Used inside the maze.h code 
        

        virtual void show(bool closed, const Position &parent);

        static Position begin();
        static Position end();

        void test(); // This function was not being used anywhere 
        int distToParent();
        std::vector<PositionPtr> children();
        
    };
}

#endif
