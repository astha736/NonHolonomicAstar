#include <a_star.h>
#include <maze.h>


#include <robotPose.h>
#include <position.h>

using namespace std;
using namespace ecn;

// a node is a x-y position, we move from 1 each time
// white -> free
// black -> blocked

int main( int argc, char **argv )
{
    int x_begin = 0, y_begin = 0, thetaBegin = 0;
    int x_end = 27,  y_end = 15,  thetaEnd = 45 ;

    int rightVelBegin = 0, rightVelEnd = 0;
    int leftVelBegin = 0, leftVelEnd = 0;

    // load file
    std::string filename = "maze_basic_3.png";
    if(argc == 2)
        filename = std::string(argv[1]);

    // let Point know about this maze
    Point::maze.load(filename);

    Position beginPosition(x_begin, y_begin, thetaBegin);
    Position endPosition(x_end, y_end, thetaEnd);

    RobotPose beginRobotPose(rightVelBegin,leftVelBegin, beginPosition);
    RobotPose endRobotPose(rightVelEnd,leftVelEnd, endPosition);

    cout << "beginRobotPose: " << beginRobotPose << std::endl;
    cout << "endRobotPose: " << endRobotPose << std::endl;


    // call A* algorithm
    if(!beginRobotPose.isFree()){
        cout<<"wrong begin point"<<endl;
        return 0;
    }
    if(!endRobotPose.isFree()){
        cout<<"wrong end point"<<endl;
        return 0;
    }

    //    cout << Point::maze.isFree(x_end,y_end) << std::endl;

    ecn::Astar(beginRobotPose, endRobotPose);

    // save final image
    Point::maze.saveSolution("result_27_15_45");
    cv::waitKey(0);

}
