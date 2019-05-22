#ifndef POINT_H
#define POINT_H

#include <cstdlib>
#include <iostream>
#include <math.h>
#include <maze.h>

namespace ecn
{

class Point
{
public:

    int x, y;
    static Maze maze;

    Point() {}
    Point(int _x, int _y): x(_x), y(_y) {}

    // Point p1 = p2; => p1.operator(p2)
    void operator=(const Point &p)
    {
        x = p.x;
        y = p.y;
    }

    friend std::ostream& operator<<(std::ostream& out, const Point& p)
    {
        out << "(" << p.x << ", " << p.y << ")";
        return out;
    }

    // 2 positions are equal if they have the same x and y
    bool is(const Point &other)
    {
        return x == other.x && y == other.y;
    }

    bool isFree();

    static Point begin();
    static Point end();
        
};

}

#endif // POINT_H
