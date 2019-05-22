#include <point.h>

using std::min;
using std::max;

namespace ecn
{

// A Static Variable defined outside the class definition
// otherwise an error undefined reference to `ecn::Point::maze'

Maze Point::maze;

bool Point::isFree(){
      if(maze.isFree(x, y)){
          return true;
      }
      return false;
   }

// Finds the first first free cell of the maze
Point Point::begin(){
    cv::Mat im = maze.im;
    Point ret;
    for(ret.y=0;ret.y<im.rows; ++ret.y)
    {
        for(ret.x=0;ret.x<im.cols;++ret.x)
        {
            if(maze.isFree(ret.x, ret.y))
            {
                return ret;
            }
        }
}
return ret;
}

// Finds the las free cell of the maze
Point Point::end()
{
    cv::Mat im = maze.im;
    Point ret;
    for(ret.y=im.rows-1;ret.y> 0; --ret.y)
    {
        for(ret.x=im.cols-1;ret.x>0;--ret.x)
        {
            if(maze.isFree(ret.x, ret.y))
            {
                return ret;
            }
        }
    }
    return ret;
}


}
