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

    struct pairVel
    {
        float right;
        float left;
        pairVel (float _right,float _left): right(_right), left(_left){}
    };

    float rightWheelVel;    // right wheel velocity
    float leftWheelVel;     // left wheel velocity
    float distance = 0;         // distance of this node from its parent

    static constexpr float rWheel = 1; // wheel radius
    static constexpr float tGauge = 1; // track gauge
    static constexpr float timeStep = 1; // time step

    static constexpr float wheelVelocityMax = 10; // max angular velocity for any wheel
    static constexpr float wheelVelocityMin = -10; // min angular velocity for any wheel
    static constexpr float wheelVelocityTolerance = 1; // angular velocity tolerance to be used in comparison

    static constexpr float velocityIncrementLimit = 5; // limit of +-rightWheelVel and leftWheelVel from current state
    static constexpr float velocityIncrementStep = 1;

    RobotPose(float _rightWheelVel, float _leftWheelVel, Position _p ) : Position(_p.x,_p.y,_p.theta) {
        rightWheelVel = _rightWheelVel;
        leftWheelVel = _leftWheelVel;
    }

    RobotPose(float _rightWheelVel, float _leftWheelVel, float _x, float _y, float _theta) : Position(_x, _y, _theta) {
        rightWheelVel = _rightWheelVel;
        leftWheelVel = _leftWheelVel;
    }

    RobotPose(pairVel _vel,Position _p ):RobotPose(_vel.right, _vel.left,_p){}
    RobotPose(pairVel _vel,Position _p,float _dist ):RobotPose(_vel.right, _vel.left,_p){
        distance = _dist;
    }

    void operator=(const RobotPose &_p)
    {
        x = _p.x;
        y = _p.y;
        theta = _p.theta;
        rightWheelVel = _p.rightWheelVel;
        leftWheelVel = _p.leftWheelVel;
    }

    // Generate a set of choices of wheel velocity
    vector<pairVel>  generateVelChoices();

    // Converting wheel velocites to linear and angular velocities stored in robotVelocity
    // for a (2,0) robot <linear, angular>
    static pair<float,float> robotVelocity(pairVel _vel_wheel);


    // method that takes v, w and generates Position we can reach (assuming 1 sec time step)
    Position getNextPosition(pair<float,float> _linVel_angVel);


    // Suggest how the object of such data type should be printed
    friend std::ostream& operator<<(std::ostream& _out, const RobotPose& _p)
    {
        _out << "(" << _p.rightWheelVel << "," << _p.leftWheelVel  << "," << _p.x << "," << _p.y << "," << _p.theta << ")" ;
        return _out;
    }

    // check of two RobotPose are same or not
    bool is(const RobotPose &_other);

    // getter to return distance attribute (distance between current node and its parent)
    float distToParent();
    // vector of unique_ptr to children
    std::vector<RobotPosePtr> children();

    // calculates the distance traveled by robot travelling at wheel velocity pair
    // will be used to calculate the distance attribute of the children
    float distTravelled(pairVel _wheelVelocityPair);

    //heuristic
    float h(const RobotPose &_goal, bool useManhattan);

};
}

#endif // ROBOTPOSE_H
