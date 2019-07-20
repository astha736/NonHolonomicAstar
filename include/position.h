#ifndef POSITION_H
#define POSITION_H

#include <cstdlib>
#include <iostream>
#include <math.h>
#include <point.h>
#include <cmath>


#include <vector>
#include <memory>
#include <maze.h>
#include <param.h>
//using namespace std;

namespace ecn
{
    class Position
    {

    protected:
//        // tolerances along x,y and theta to be ignored whiel some comparison
//        static constexpr float xTolerance = 1; //cm
//        static constexpr float yTolerance = 1;
//        static constexpr float thetaTolerance = M_PI/36; // degree

//        // for future use
//        static constexpr float scaleFactor = 1; // assumption( 1 pixel = scaleFactor x 1 cm : minimum step of the robot)

    public:
        friend class Param;
        typedef std::unique_ptr<Position> PositionPtr;

        float x;
        float y;
        float theta;

        Position(){
            x = 0;
            y = 0;
            theta = 0;
        }

        // constructor from coordinates
        Position(float _x, float _y, float _theta){
            x = _x;
            y = _y;
            theta = thetaBound(_theta);
        }

        // constructor from base ecn::Point
        Position(Point _p, float _theta){
            x = _p.x;
            y = _p.y;
            theta = thetaBound(_theta);
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
        int scaleAndFloor(float _float);
        static float thetaBound(float _theta){
            if(_theta >= 0){
                return remainder(_theta,(2.0*M_PI));
            }
            else{
                return ((2.0*M_PI) - remainder(abs(_theta),(2.0*M_PI)));
            }
        }
        float euclideanDist(Position _other);

    };
}

#endif

//        float distFromPosition(const Position &p);
//        static float convertDegreeToRadian(int _deg);
