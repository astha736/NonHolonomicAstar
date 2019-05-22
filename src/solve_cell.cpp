#include <a_star.h>
#include <maze.h>


#include <robotPose.h>
#include <position.h>

using namespace std;
using namespace ecn;

// a node is a x-y position, we move from 1 each time

int main( int argc, char **argv )
{
    int x_begin = 0;
    int x_end = 0;

    int y_begin = 0;
    int y_end = 0;

    // let theta be between 0 to 360
    int theta_begin = 0;
    int theta_end = 0;

    int lin_vel_begin = 0;
    int ang_vel_begin = 0;

    int lin_vel_end = 0;
    int ang_vel_end = 0;

    // load file
    std::string filename = "maze_basic.png";
    if(argc == 2)
        filename = std::string(argv[1]);

    // let Point know about this maze
    Point::maze.load(filename);

    Point begin_maze =  Point::begin();
    Point end_maze =  Point::end();
    cout << "begin_maze: " << begin_maze << std::endl;
    cout << "end_maze: " << end_maze << std::endl;
    // initial and end positions as Position's

    Position begin_position(begin_maze,theta_begin);
    Position end_position(end_maze,theta_end);

    //    Position begin_position(x_begin,y_begin,theta_begin);
    //    Position end_position(x_end, y_end, theta_begin);

    cout << "begin_position: " << begin_position << std::endl;
    cout << "end_position: " << end_position << std::endl;

    RobotPose begin_robotPose(lin_vel_begin,ang_vel_begin, begin_position);
    RobotPose end_robotPose(lin_vel_end,ang_vel_end, end_position);

    cout << "begin_robotPose: " << begin_robotPose << std::endl;
    cout << "end_robotPose: " << end_robotPose << std::endl;

    // // call A* algorithm
    // ecn::Astar(begin_robotPose, end_robotPose);

    // save final image
    // Point::maze.saveSolution("cell");
    cv::waitKey(0);

}
