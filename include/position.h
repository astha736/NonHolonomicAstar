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

    protected:
        // tolerances along x,y and theta to be ignored whiel some comparison
        static constexpr float xTolerance = 0.5; //cm
        static constexpr float yTolerance = 0.5;
        static constexpr float thetaTolerance = 15; // degree

        // for future use
        static constexpr float scaleFactor = 1; // assumption( 1 pixel = scaleFactor x 1 cm : minimum step of the robot)

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
        Position(Point _p, float _theta){
            x = _p.x;
            y = _p.y;
            theta = _theta;
        }


        // assignmnet operator
        void operator=(const Position &_p)
        {
            x = _p.x;
            y = _p.y;
            theta = _p.theta;
        }

        // cout template
        friend std::ostream& operator<<(std::ostream& _out, const Position& _p)
        {
            _out << "(" << _p.x << "," << _p.y << "," << _p.theta << ")" ;
            return _out;
        }

        // Check: two positions are equal - with added tolerances along x, y and theta
        bool is(const Position &_other);

        // TODO: check the implementation
        bool isFree();

        // Functions that are used for visualization purposes
        void print(const Position &_parent);
        void start(); // Used inside the maze.h code
        void show(bool _closed, const Position &_parent);

    };
}

#endif

//        float distFromPosition(const Position &p);
//        static float convertDegreeToRadian(int _deg);
