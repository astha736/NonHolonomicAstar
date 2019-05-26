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
        // A constructor for pairVel: makes it easier to create a variable
        pairVel (float _right,float _left): right(_right), left(_left){}
    };

    //MAYBE MAKE ALL INT

    float r_vel; // right angular velocity
    float l_vel; // left angular velocity
//    float dist;
    float distance; // gives no error: Astha


    // const: since we don't want anyone to change this attribute
    // static: so that only one copy of the variable is made
    const static int vel_increment_limit = 2; // limit of +-r_vel and l_vel from current state
    static constexpr float rWheel = 1; // wheel radius
    static constexpr float tGauge = 1; // track gauge
    static constexpr float timeStep = 1; // time step


    // when using a float or double kind of variable use a "constexpr"
    static constexpr float vel_max = 10; // max angular velocity for any wheel
    static constexpr float vel_min = -10; // min angular velocity for any wheel
    static constexpr float vel_tol = 0; // angular velocity tolerance to be used in comparison
//    static constexpr float vel_parent_tol = 0; // range in which child velocities should lie wrt parent

    static constexpr float velocityIncrementStep = 1;


    // constructor from coordinates

    // constructor from base ecn::Point
    RobotPose(float _r_vel, float _l_vel, Position _p ) : Position(_p.x,_p.y,_p.theta) {
        r_vel = _r_vel;
        l_vel = _l_vel;
    }

    RobotPose(float _r_vel, float _l_vel, float _x, float _y, float _theta) : Position(_x, _y, _theta) {
        r_vel = _r_vel;
        l_vel = _l_vel;
    }

    RobotPose(pairVel _vel,Position _p ):RobotPose(_vel.right, _vel.left,_p){}
    RobotPose(pairVel _vel,Position _p,float _dist ):RobotPose(_vel.right, _vel.left,_p){
        distance = _dist;
    }

    // assignmnet operator
    void operator=(const RobotPose &p)
    {
        x = p.x;
        y = p.y;
        theta = p.theta;
        r_vel = p.r_vel;
        l_vel = p.l_vel;
    }

    // Generate a set of choices of wheel velocity
    vector<pairVel>  generateVelChoices();

    // Converting wheel velocites to linear and angular velocities stored in robotVelocity
    // for a (2,0) robot <linear, angular>
    static pair<float,float> robotVelocity(pairVel _vel_wheel);


    // method that takes v, w and generates Position we can reach (assuming 1 sec time step)
    Position getNextPosition(pair<float,float> _vel_vel_omega);


    // Suggest how the object of such data type should be printed
    friend std::ostream& operator<<(std::ostream& out, const RobotPose& p)
    {
        out << "(X:" << p.x << ",Y: " << p.y << ",theta:" << p.theta << ",r_vel:" << p.r_vel << ",l_vel:" << p.l_vel << ")" ;
        return out;
    }

    // check of two RobotPose are same or not
    bool is(const RobotPose &other);

    // Functions that are used for visualization purposes
    virtual void print(const RobotPose &parent);
    void start(); // Used inside the maze.h code
    virtual void show(bool closed, const RobotPose &parent);

    //   Inherited from Position.h
    //    bool isFree();
    //    double h(const RobotPose &goal, bool use_manhattan);

    float distToParent();
    std::vector<RobotPosePtr> children();
    float distTravelled(RobotPose::pairVel wheel_vel);
    float h(const RobotPose &goal); //heuristic

    // This function was not being used anywhere
    // just kept it incase we find some use of this
    //    void test();


};
}

#endif // ROBOTPOSE_H
