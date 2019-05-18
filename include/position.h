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
    class Position : public Point
    {
        typedef std::unique_ptr<Position> PositionPtr;

    public:

        int theta;
        // constructor from coordinates
        Position(int _x, int _y, int _theta) : Point(_x, _y) {
            theta = _theta;
        }

        // constructor from base ecn::Point
        Position(ecn::Point p, int _theta) : Point(p.x, p.y) {
            theta = _theta;
        }

        int distToParent();

        std::vector<PositionPtr> children();
        
    };
}

#endif
