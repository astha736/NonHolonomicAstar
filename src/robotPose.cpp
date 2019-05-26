#include <robotPose.h>
#include <point.h>
#include <position.h>

namespace ecn
{

vector<RobotPose::pairVel>  RobotPose::generateVelChoices(){
    // creates 2 different vector containing the possible choices that the two wheels can have.
    int scale = -1*vel_increment_limit;
    vector<float> r_vel_choice;
    vector<float> l_vel_choice;
    float temp_r_vel;
    float temp_l_vel;
    while(scale <= vel_increment_limit ){
        temp_r_vel = r_vel+scale;
        temp_l_vel = l_vel+scale;

        if(temp_r_vel > vel_max || temp_r_vel < vel_min); // do nothing
        else {
            r_vel_choice.push_back(temp_r_vel);
        }
        if(temp_l_vel > vel_max || temp_l_vel < vel_min); // do nothing
        else {
            l_vel_choice.push_back(temp_l_vel);
        } // LOOK HERE
        scale += velocityIncrementStep; // increase the scale by velocity step
    }

    // need to create the wheelVelVectorPair which can be directly be used to create valid childrens
    vector<RobotPose::pairVel> ret; // return variable

    for(int i =0; i < r_vel_choice.size(); i++){
        for(int j=0; j < l_vel_choice.size(); j++){
            ret.push_back(RobotPose::pairVel(r_vel_choice[i],l_vel_choice[j]));

        }
    }
    return ret;
}

pair<float,float> RobotPose::robotVelocity(RobotPose::pairVel _vel_wheel){
    pair<float,float> returnVel;
    returnVel.first = (rWheel*_vel_wheel.right + rWheel*_vel_wheel.left)/2;
    returnVel.second = (rWheel*_vel_wheel.right - rWheel*_vel_wheel.left)/(2*tGauge);
    return returnVel;
}

// TODO
Position RobotPose::getNextPosition(pair<float,float> _vel_vel_omega){

    // pair.first = linear velocity
    float xPos = x+_vel_vel_omega.first*timeStep*cos(theta);
    float yPos = y+_vel_vel_omega.first*timeStep*sin(theta);

    // pair.second = angular velocity
    float thetaPos = theta + _vel_vel_omega.second;
    return Position(xPos, yPos, thetaPos);
}

std::vector<RobotPose::RobotPosePtr> RobotPose::children()
{
    // TODO: write this print correctly
    std::vector<RobotPosePtr> generated;
    // this method should return  all positions reachable from this one

    // based on the vector of acceptable velocity generate different children
    vector<RobotPose::pairVel> velChoices;
    velChoices = RobotPose::generateVelChoices();

    // for all the possible generated combination create children
    for(int i=0; i < velChoices.size(); i++){

        // step1. calculate lin-vel and ang-vel
        pair<float,float> tempVelWheel = RobotPose::robotVelocity(velChoices[i]);

        // step2. calculate the (x,y,theta) of the possible child node  
        Position tempPosition  = RobotPose::getNextPosition(tempVelWheel);
        
        // step3. check if the child node is free(valid) or not
        if(!tempPosition.isFree()) continue; // skip this iteration if not free

        // step4. calculate distance of the child from this node(parent)
        float tempdist = distTravelled(velChoices[i]);

        // step5. make an object for the child with a unique_ptr
        generated.push_back(std::make_unique <RobotPose>(velChoices[i],tempPosition,tempdist));
    }
    return generated;
}

float RobotPose::distToParent()
{
    // TODO: write this print correctly
    // in cell-based motion, the distance to the parent is always 1
    //return dist;
    return distance;
}


// check of two RobotPose are same or not
bool RobotPose::is(const RobotPose &other)
{
    if((other.x - x_tol <= x <= other.x + x_tol) &&
            (other.y - y_tol <= y <= other.y + y_tol) &&
            (other.theta - theta_tol <= theta <= other.theta + theta_tol) &&
            (other.l_vel - vel_tol <= l_vel <= other.l_vel + vel_tol) &&
            (other.r_vel - vel_tol <= r_vel <= other.r_vel + vel_tol))
    {
        return 1;
    }
    return 0;
}


// TODO: check if parent needs to be typecasted to int
void RobotPose::print(const RobotPose &parent)
{
    int x_incr(0), y_incr(0);
    // may need to change incase we assume anything other than 1 pix <=> 1 cm

    // check if the parent is different from child
    if(x - parent.x)
        x_incr = x - parent.x > 0 ? 1 : -1;
    else
        y_incr = y - parent.y > 0 ? 1 : -1;

    int k = 1;
    while(parent.x + k*x_incr != x || parent.y + k*y_incr != y)
    {
        Point::maze.passThrough(parent.x + k*x_incr,
                                parent.y + k*y_incr);
        k++;
    }

    Point::maze.passThrough(x, y);

}

// TODO: check if parent needs to be typecasted to int
// used in the maze function, have to check and update 
// TODO: write this print correctly
void RobotPose::start()
{
    Point::maze.write(x, y);
}

// TODO: check if parent needs to be typecasted to int
// online print, color depends on closed / open set
// TODO: write this print correctly
void RobotPose::show(bool closed, const RobotPose & parent)
{
    const int b = closed?255:0, r = closed?0:255;
    if(x != parent.x)
        for(int i = min(x, parent.x); i <= max(x, parent.x);++i)
            Point::maze.write(i, y, r, 0, b, false);
    else
        for(int j = min(y, parent.y); j <= max(y, parent.y);++j)
            Point::maze.write(x, j, r, 0, b, false);
    Point::maze.write(x, y, r, 0, b);
}

float RobotPose::distTravelled(RobotPose::pairVel wheel_vel){
    // returns the distance travelled by the robot
    // distance taken as the average of wheel rotations during motion from parent to child
    // in 1 time-step

    return ((wheel_vel.left)*timeStep + (wheel_vel.right)*timeStep)/2;
}








} // END of namespace
