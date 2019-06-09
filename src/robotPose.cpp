#include <robotPose.h>
#include <point.h>
#include <position.h>

namespace ecn
{

RobotPose RobotPose::goalPose(0,0,0,0,0);

vector<RobotPose::pairVel>  RobotPose::generateVelChoices(){
    // creates 2 different vector containing the possible choices that the two wheels can have.
    int scale = -1*velocityIncrementLimit;
    vector<float> rightWheelVel_choice;
    vector<float> leftWheelVel_choice;
    float temp_rightWheelVel;
    float temp_leftWheelVel;
    while(scale <= velocityIncrementLimit ){
        temp_rightWheelVel = rightWheelVel+scale;
        temp_leftWheelVel = leftWheelVel+scale;

        if(temp_rightWheelVel > wheelVelocityMax || temp_rightWheelVel < wheelVelocityMin); // do nothing
        else {
            rightWheelVel_choice.push_back(temp_rightWheelVel);
        }
        if(temp_leftWheelVel > wheelVelocityMax || temp_leftWheelVel < wheelVelocityMin); // do nothing
        else {
            leftWheelVel_choice.push_back(temp_leftWheelVel);
        }
        scale += velocityIncrementStep; // increase the scale by velocity step
    }

    // need to create the wheelVelVectorPair which can be directly be used to create valid childrens
    vector<RobotPose::pairVel> ret; // return variable

//    cout << *this << endl;
    for(int i =0; i < rightWheelVel_choice.size(); i++){
        for(int j=0; j < leftWheelVel_choice.size(); j++){
            ret.push_back(RobotPose::pairVel(rightWheelVel_choice[i],leftWheelVel_choice[j]));
//            cout << rightWheelVel_choice[i] <<":" << leftWheelVel_choice[j] << endl;
        }
    }
    return ret;
}

pair<float,float> RobotPose::robotVelocity(RobotPose::pairVel _vel_wheel){
    pair<float,float> returnVel;
    returnVel.first = ((rWheel*_vel_wheel.right + rWheel*_vel_wheel.left)/2); //*(M_PI/180);
    returnVel.second = (rWheel*_vel_wheel.right - rWheel*_vel_wheel.left)/(2*tGauge);
    return returnVel;
}

Position RobotPose::getNextPosition(pair<float,float> _linVel_angVel, float _timeStep){

    // pair.first = linear velocity
    float xPos = x+_linVel_angVel.first*_timeStep*cos(theta);// * M_PI/180);
    float yPos = y+_linVel_angVel.first*_timeStep*sin(theta); //* M_PI/180);

    // pair.second = angular velocity
    float thetaPos = theta + _linVel_angVel.second*_timeStep;
    return Position(xPos, yPos, thetaPos);
}

float RobotPose::distTravelled(pairVel _wheelVelocityPair, float _timeStep){
    // returns the distance travelled by the robot
    // distance taken as the average of wheel rotations during motion from parent to child
    // in 1 time-step
    return (abs((_wheelVelocityPair.left)*_timeStep) + abs((_wheelVelocityPair.right)*_timeStep))/2;
}

////
float RobotPose::calcTimeStep(float _hDistance){
    if(_hDistance >= 12*M_PI){
        return bigTimeStep;
    }
    else if (_hDistance < 12*M_PI && _hDistance >= 8*M_PI ){
        return (3/4)*bigTimeStep;
    }
    else if (_hDistance < 8*M_PI && _hDistance >= 4*M_PI ){
        return (2/4)*bigTimeStep;
    }
    else {
        return (1/4)*bigTimeStep;
    }
}

void RobotPose::setGoalPose(RobotPose _goalPose){
     goalPose = _goalPose;
}

std::vector<RobotPose::RobotPosePtr> RobotPose::children()
{
    // this method should return  all positions reachable from this one
    std::vector<RobotPosePtr> generated;

    // based on the vector of acceptable velocity generate different children
    vector<pairVel> velChoices;
    velChoices = generateVelChoices();

    // for all the possible generated combination create children
    for(int i=0; i < velChoices.size(); i++){

        // step1. calculate lin-vel and ang-vel
        pair<float,float> tempVelWheel = RobotPose::robotVelocity(velChoices[i]);


//        // step1.1 make the goal robot pose
//        RobotPose goalPose(goalWheelVelocities, goalPosition);

        // step1.2 find the heuristic distance to this goal pose
//        float heuristicDistance = getHeuristicDistance();
        float heuristicDistance = this->h(goalPose,true);

        // step1.3 calculate the time step based on the heuristic
        float tempTimeStep = calcTimeStep(heuristicDistance);

        // step2. calculate the (x,y,theta) of the possible child node  
        Position tempPosition  = RobotPose::getNextPosition(tempVelWheel, tempTimeStep);
        
        // step3. check if the child node is free(valid) or not
        if(!tempPosition.isFree()) continue; // skip this iteration if not free

        // step4. calculate distance of the child from this node(parent)
        float tempdist = distTravelled(velChoices[i], tempTimeStep);

        // step5. make an object for the child with a unique_ptr
        generated.push_back(std::make_unique <RobotPose>(velChoices[i],tempPosition,tempdist,tempTimeStep));
    }
//    for(int i = 0; i < generated.size(); i++){
//        cout << *generated[i] << endl;
//    }
    return generated;
}

float RobotPose::distToParent()
{
    return distance;
}



bool RobotPose::is(const RobotPose &_other)
{
    // check of two RobotPose are same or not
    Position othePosition = _other;
    Position* nodePosition = this;

    bool clauseA = nodePosition->is(othePosition) ;
    bool clauseB = ((_other.leftWheelVel - wheelVelocityTolerance) <= this->leftWheelVel)
            && (this->leftWheelVel <= (_other.leftWheelVel + wheelVelocityTolerance)) ;
    bool clauseC = ((_other.rightWheelVel - wheelVelocityTolerance) <= this->rightWheelVel)
            && (this->rightWheelVel <= (_other.rightWheelVel + wheelVelocityTolerance)) ;

    if(clauseA && clauseB && clauseC)
    {
        return 1;
    }
    return 0;
}


float RobotPose::h(const RobotPose &_goal, bool useManhattan)
{
    float thetaDispTemp = _goal.theta - theta;
    float thetaDisp = min(abs(thetaDispTemp),360 - abs(thetaDispTemp));
    float wheelSpinTheta = thetaDisp*tGauge/(2*rWheel);

    float x_dist = (x - _goal.x);
    float y_dist = (y - _goal.y);

    float wheelSpinStraight = sqrt(pow(x_dist,2) + pow(y_dist,2))/rWheel;

    return wheelSpinStraight + wheelSpinTheta;
}


} // END of namespace
