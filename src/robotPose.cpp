#include <robotPose.h>
#include <point.h>
#include <position.h>

namespace ecn
{

RobotPose RobotPose::goalPose(0,0,0,0,0);

float RobotPose::distToParent()
{
    return distance;
}

void RobotPose::setGoalPose(RobotPose _goalPose){
     goalPose = _goalPose;
}

pair<float,float> RobotPose::robotVelocity(RobotPose::pairVel _vel_wheel){
    pair<float,float> returnVel;
    returnVel.first = ((rWheel*_vel_wheel.right + rWheel*_vel_wheel.left)/2); //*(M_PI/180);
    returnVel.second = (rWheel*_vel_wheel.right - rWheel*_vel_wheel.left)/(2*tGauge);
    return returnVel;
}


void RobotPose::print(const RobotPose &_parent)
{
//    std::cout << "******** ((())))))) " << std::endl;
//    std::cout << "parent: " << _parent << std::endl;
//    std::cout << "child: " << *this << std::endl;
//    std::cout << "timestep: " << this->timeStep << std::endl;
//    Position tempPosition = _parent;
//    pair<float,float> linVel_angVel = robotVelocity(pairVel(this->rightWheelVel, this->leftWheelVel));

    Position tempPosition;
    pair<float,float> linVel_angVel = robotVelocity(pairVel(this->rightWheelVel, this->leftWheelVel));

    float timeIndex = 0;
    while(timeIndex < this->timeStep){
            timeIndex = timeIndex + obstacleCheckInterval;
            tempPosition = getNewStepPosition(Position(_parent.x, _parent.y, _parent.theta) , linVel_angVel, timeIndex );
//            if(!tempPosition.isFree()) continue;
            int xTemp = scaleAndFloor(tempPosition.x);// floor(x/scaleFactor);
            int yTemp = scaleAndFloor(tempPosition.y);// floor(y/scaleFactor);
            Point::maze.passThrough(xTemp,yTemp);

//            std::cout << "********" << std::endl;
//             std::cout << "delT: " << delT << std::endl;
//            std::cout <<"tempPosition"  << tempPosition << std::endl;
        }
    }
}


pair<bool,Position> RobotPose::validPathPosition(Position _startPosition, pair<float,float> _linVel_angVel, float _timeStep ){

    Position tempPosition;
    float timeIndex = 0;
        while(timeIndex < _timeStep){
            timeIndex = timeIndex + obstacleCheckInterval;
            tempPosition = getNewStepPosition(_startPosition, _linVel_angVel, timeIndex);
        if(!tempPosition.isFree()) return make_pair(false,tempPosition);
    }
    return make_pair(true,tempPosition);
}

Position RobotPose::getNewStepPosition(Position _startPosition, pair<float,float> _linVel_angVel, float _time ){
    // don't know what is the correct approx theta -> discuss

    // pair.first = linear velocity
    float v = _linVel_angVel.first;
    float w = _linVel_angVel.second;

    float thetaPos = _startPosition.theta + w*_time;

    // probably this theta calcilation is not correvt
    float xPos ;
    float yPos ;
    if(w != 0){
        xPos = _startPosition.x+ (v*cos( w*_time))/w;
        yPos = _startPosition.y+ (v*sin( w*_time))/w;
    }
    else{
        xPos = _startPosition.x+ v*cos(thetaPos)*_time;
        yPos = _startPosition.y+ v*sin(thetaPos)*_time;
    }

    // pair.second = angular velocity
    return Position(xPos, yPos, thetaPos);

}

float RobotPose::distTravelled(pairVel _wheelVelocityPair, float _timeStep){
    // returns the distance travelled by the robot
    // distance taken as the average of wheel rotations during motion from parent to child
    // in 1 time-step
    return (abs((_wheelVelocityPair.left)*_timeStep) + abs((_wheelVelocityPair.right)*_timeStep))/2;
}

float RobotPose::calcTimeStep(float _hDistance){
//    float noOfWheelRevolution = _hDistance/M_PI;
//    if(noOfWheelRevolution >= 12){
//        return bigTimeStep;
//    }
//    else if (noOfWheelRevolution < 12 && noOfWheelRevolution >= 8){
//        return ((3*bigTimeStep)/4);
//    }
//    else if (noOfWheelRevolution < 8 && noOfWheelRevolution >= 4 ){
//        return ((2*bigTimeStep)/4);
//    }
//    else {
//        return ((1*bigTimeStep)/4);
//    }
    return 0.5;
}

vector<RobotPose::pairVel>  RobotPose::generateVelChoices(){
    // creates 2 different vector containing the possible choices that the two wheels can have.
    float scale = -1.0*velocityIncrementLimit;
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

    for(int i =0; i < rightWheelVel_choice.size(); i++){
        for(int j=0; j < leftWheelVel_choice.size(); j++){
            ret.push_back(RobotPose::pairVel(rightWheelVel_choice[i],leftWheelVel_choice[j]));
        }
    }
    return ret;
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
        pair<float,float> tempVelWheel = robotVelocity(velChoices[i]);

        // step1.2 find the heuristic distance to this goal pose
        float heuristicDistance = this->h(goalPose,true);

        // step1.3 calculate the time step based on the heuristic
        float tempTimeStep = calcTimeStep(heuristicDistance);

        // step2. calculate the (x,y,theta) of the possible child node
         std::pair<bool,Position> tempPairBoolPos = validPathPosition(*this,tempVelWheel, tempTimeStep);
        
        // step3. check if the child node is free(valid) or not
        if(!tempPairBoolPos.first) continue; // skip this iteration if not free

        // step4. calculate distance of the child from this node(parent)
        float tempdist = distTravelled(velChoices[i], tempTimeStep);

        // step5. make an object for the child with a unique_ptr
        generated.push_back(std::make_unique <RobotPose>(velChoices[i],tempPairBoolPos.second,tempdist,tempTimeStep));
    }
    return generated;
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
//    float thetaDispTemp = _goal.theta - theta;
//    float thetaDisp = min(abs(thetaDispTemp), float(2*M_PI - abs(thetaDispTemp)));
//    float wheelSpinTheta = thetaDisp*tGauge/(2*rWheel);

//    float x_dist = (x - _goal.x);
//    float y_dist = (y - _goal.y);

//    float wheelSpinStraight = sqrt(pow(x_dist,2) + pow(y_dist,2))/rWheel;

//    return wheelSpinStraight + wheelSpinTheta;

        float x_dist = (x - _goal.x);
        float y_dist = (y - _goal.y);

        float wheelSpinStraight = sqrt(pow(x_dist,2) + pow(y_dist,2));
        return wheelSpinStraight;



}


} // END of namespace
