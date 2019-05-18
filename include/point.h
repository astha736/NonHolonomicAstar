#ifndef POINT_H
#define POINT_H

#include <cstdlib>
#include <iostream>
#include <math.h>

namespace ecn
{

/*  Forward declaration, as Maze and Point are both interdependent. 
    Check: https://stackoverflow.com/questions/16663207/c-how-to-create-classes-that-are-interdependent 
*/
class Maze;

class Point
{
public:

    Point() {}
    Point(int _x, int _y): x(_x), y(_y) {}

    // Point p1 = p2; => p1.operator(p2)
    void operator=(const Point &p)
    {
        x = p.x;
        y = p.y;
    }

    // prints the grid with all positions from parent
    virtual void print(const Point &parent);

    void start();
    void test();

    virtual void show(bool closed, const Point &parent);

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

    /* 
        h: is the heuristic function, will provide the heuristic of
        this node (x,y) from the goal node 
    */
    double h(const Point &goal, bool use_manhattan)
    {
        if(use_manhattan)
            return  abs(x-goal.x) + abs(y-goal.y);
        return 1.5*sqrt((x-goal.x)*(x-goal.x) + (y-goal.y)*(y-goal.y));
    }

    int x, y;
    static Maze maze;
};

}

#endif // POINT_H
