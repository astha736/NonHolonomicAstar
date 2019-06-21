#ifndef ROBOTPOSE_H
#define ROBOTPOSE_H

#include <cstdlib>
#include <iostream>
#include <math.h>
#include <position.h>


#include <vector>
#include <memory>
#include <param.h>

using namespace std;

namespace ecn
{
class RobotPose: public Position
{
    typedef std::unique_ptr<RobotPose> RobotPosePtr;

public:

    friend class Param;
    struct pairVel
    {
        float right;
        float left;
        pairVel (float _right,float _left): right(_right), left(_left){}
    };

    float rightWheelVel;    // right wheel velocity
    float leftWheelVel;     // left wheel velocity
    float distance = 0;         // distance of this node from its parent
    float timeStep = 0; // the variable timestep (default value = 0s)
//    float obstacleCheckInterval = 0.2; // del-time step to prevent leaping over obstacles and printing

    static RobotPose goalPose;

    static float goalRadius;
    static std::vector<float> radiusRegionLimitVector; // vector containing the heuristic distance limits for determining the timestep
    static std::vector<float> intervalVector; // vector containing these smaller intervals
    static float KEFinal;

    // construtor
    RobotPose(float _rightWheelVel, float _leftWheelVel, Position _p) : Position(_p.x,_p.y,_p.theta) {
        rightWheelVel = _rightWheelVel;
        leftWheelVel = _leftWheelVel;
    }
    // construtor
    RobotPose(float _rightWheelVel, float _leftWheelVel, float _x, float _y, float _theta) : Position(_x, _y, _theta) {
        rightWheelVel = _rightWheelVel;
        leftWheelVel = _leftWheelVel;
    }
    // construtor
    RobotPose(pairVel _vel,Position _p ) : RobotPose(_vel.right, _vel.left,_p){}

    // construtor
    RobotPose(pairVel _vel,Position _p, float _dist, float _timeStep) : RobotPose(_vel.right, _vel.left,_p){
        distance = _dist;
        timeStep = _timeStep;
    }

    // operator overloading
    void operator=(const RobotPose &_p)
    {
        x = _p.x;
        y = _p.y;
        theta = _p.theta;
        rightWheelVel = _p.rightWheelVel;
        leftWheelVel = _p.leftWheelVel;
    }

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

    // Generate a set of choices of wheel velocity
    vector<pairVel>  generateVelChoices();

    // Converting wheel velocites to linear and angular velocities stored in robotVelocity for a (2,0) robot <linear, angular>
    static pair<float,float> robotVelocity(pairVel _vel_wheel);

    // calculates the distance traveled by robot travelling at wheel velocity pair
    // will be used to calculate the distance attribute of the children
    float distTravelled(pairVel _parentVelocityPair, pairVel _wheelVelocityPair, float _timeStep);

    //heuristic
    float h(const RobotPose &_goal, bool useManhattan);

    void setGoalPose(RobotPose _goalPose);
    void fillIntervalVector();

    // calculate the time step on basis of the heuristic distance
    pair<float,float> calcTimeStep(float _hDistance);

    // calculate newPosition and its validity from start, velovities and timestep
    bool validPathPosition(const RobotPose &_startPosition, Position _tempChildPosition, pair<float,float> _linVel_angVel, float _timeStep );

    // implementing dicrete integration
    Position getNewStepPosition(const RobotPose &_startPosition, pair<float,float> _linVel_angVel, float _delTime );

    // prints all the maze cell visited in between this node and its parents
    void print(const RobotPose &_parent);

    bool isChildHealthy(const RobotPose &_parent, Position _tempChildPosition);

    void fillRadiusRegionLimitVector();

};
}

#endif // ROBOTPOSE_H
