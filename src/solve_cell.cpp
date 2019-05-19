#include <a_star.h>
#include <maze.h>
//#include <robotPose.h>
#include <point.h>

using namespace std;
using namespace ecn;

// a node is a x-y position, we move from 1 each time

int main( int argc, char **argv )
{
    int theta_start = 0;
    int theta_end = 0;

    int lin_vel_start = 0;
    int ang_vel_start = 0;

    int lin_vel_end = 0;
    int ang_vel_end = 0;

    // load file
    std::string filename = "maze.png";
    if(argc == 2)
        filename = std::string(argv[1]);

    // let Point know about this maze
    Point::maze.load(filename);

    // initial and goal positions as Position's

    Point start_point = Point::begin();
    Point end_point = Point::end();

    // Position start_position(start_point, theta_start); 
    // Position goal_position(end_point, theta_end);


    // RobotPose start_robotPose(lin_vel_start,ang_vel_start, start_position); 
    // RobotPose goal_robotPose(lin_vel_end,ang_vel_end, goal_position);

    // // call A* algorithm
    // ecn::Astar(start_robotPose, goal_robotPose);

    // save final image
    // Point::maze.saveSolution("cell");
    cv::waitKey(0);

}
