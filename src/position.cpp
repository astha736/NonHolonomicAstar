#include <position.h>

#include <math.h>
#include <point.h>

using namespace std;

namespace ecn
{


bool Position::is(const Position &_other)
{
    // Check: two positions are equal - with added tolerances along x, y and theta
    bool clauseX    = ((_other.x - xTolerance) <= this->x)
            &&  (this->x <= (_other.x + xTolerance));
    bool clauseY    = ((_other.y - yTolerance) <= this->y)
            &&  (this->y <= (_other.y + yTolerance));
    bool clauseTheta= ((_other.theta - thetaTolerance) <= this->theta)
            &&  (this->theta <= (_other.theta + thetaTolerance));

    if( clauseX && clauseY && clauseTheta){
        return 1;
    }
    return 0;
}

int Position::scaleAndFloor(float _float){
    return floor(_float/scaleFactor);
}

bool Position::isFree(){
    // This isFree is currently used by the RobotPose class as well
    int xTemp = scaleAndFloor(x);// floor(x/scaleFactor);
    int yTemp = scaleAndFloor(y);// floor(y/scaleFactor);
//    cout<<xTemp<<","<<yTemp<<endl;
    if(Point::maze.isFree(xTemp,yTemp)){
        return true;
    }
    return false;
    }

void Position::print(const Position &_parent)
{
    int xTemp = scaleAndFloor(x);// floor(x/scaleFactor);
    int yTemp = scaleAndFloor(y);// floor(y/scaleFactor);
    Point::maze.passThrough(xTemp,yTemp);
    cout << *this << endl;

}

void Position::start()
{
    int xTemp = scaleAndFloor(x);//floor(x/scaleFactor);
    int yTemp = scaleAndFloor(y);//floor(y/scaleFactor);
    Point::maze.write(xTemp, yTemp);
}

void Position::show(bool _closed, const Position & _parent)
{
    const int b = _closed?255:0, r = _closed?0:255;
    if(x != _parent.x)
        for(int i = floor(min(x, _parent.x)); i <= max(x, _parent.x);++i)
            Point::maze.write(i, floor(y), r, 0, b, false);
    else
        for(int j = floor(min(y, _parent.y)); j <= max(y, _parent.y);++j)
            Point::maze.write(floor(x), j, r, 0, b, false);
    Point::maze.write(floor(x), floor(y), r, 0, b);
}

} // end of cpp file












// This heurstic is currently used by the RobotPose class as well

//double Position::h(const Position &goal, const Position &starting) //, bool use_manhattan)
//{
//    // if(use_manhattan)
//    //     return  abs(x-goal.x) + abs(y-goal.y);
//    // return 1.5*sqrt((x-goal.x)*(x-goal.x) + (y-goal.y)*(y-goal.y));
//    float x_dist = (x - starting.x)/(goal.x - starting.x);
//    float y_dist = (y - starting.y)/(goal.y - starting.y);
//    float theta_dist = (theta - starting.theta)/(goal.theta - starting.x);
//    return sqrt(pow(x_dist,2) + pow(y_dist,2) + pow(theta_dist,2));

//}

// TODO: Put some values here
//float Position::distFromPosition(const Position &p){
//    //
//    return 1;
//}
