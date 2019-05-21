#ifndef MAZE_H
#define MAZE_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

namespace ecn
{

typedef std::pair<int, int> Pair;

class Maze
{
public:
    Maze() {}

    Maze(std::string _filename)
    {
      load(_filename);
    }

    void load(std::string _filename)
    {
      filename = _filename;
      // load image
      if(filename.at(0) == '/') im = cv::imread(filename, cv::IMREAD_GRAYSCALE);
      else im = cv::imread("../mazes/"+filename, cv::IMREAD_GRAYSCALE);
      cv::cvtColor(im, out, cv::COLOR_GRAY2BGR);
      std::cout << im << std::endl;

      // below lines of code are only for display and debugging 
      // cv::namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
      cv::imshow( "Display window", im ); 
    }

    Maze(int height, int width)
    {
        im = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));
    }

    bool isFree(int x, int y) const
    {
      if(x < 0 || y < 0 || x >= im.cols || y >= im.rows)
          return 0;
      return im.at<uchar>(y,x);
    }

    int height() {return im.rows;}
    int width() {return im.cols;}

    void passThrough(int x, int y)
    {
        path.push_back(cv::Point(x,y));
    }

    void dig(int x, int y)
    {
        im.at<uchar>(y,x) = 255;
    }

    void display(const std::string &name, const cv::Mat &im)
    {
        if(std::find(windows.begin(), windows.end(), name) == windows.end())
        {
            windows.push_back(name);
            cv::namedWindow(name, cv::WINDOW_NORMAL);
            cv::resizeWindow(name, 1000, (1000*im.rows)/im.cols);
        }
        cv::imshow(name, im);
    }

    void write(int x, int y, int r=0, int g=0, int b=0, bool show = true)
    {
        out.at<cv::Vec3b>(y, x) = cv::Vec3b(b, g, r);
        if(show)
        {
            display("Maze", out);
            cv::waitKey(1);
        }
    }

    void save()
    {
        cv::imwrite("../mazes/maze.png", im);
        display("Maze", im);
    }

    void saveSolution(std::string suffix)
    {
        cv::cvtColor(im, out, cv::COLOR_GRAY2BGR);
        cv::Vec3b col(0, 255, 0);

        col[1] = 0;
        for(int i = 0; i < path.size(); ++i)
        {
            col[2] = i*255/path.size();
            col[0] = 255-col[2];
            out.at<cv::Vec3b>(path[i]) = col;
        }

        // re-write black nodes just to be sure...
        for(int x = 0; x < im.cols; ++x)
        {
            for(int y = 0; y < im.rows; ++y)
            {
                if(!isFree(x,y))
                    write(x,y,0,0,0,false);
            }
        }

        int dot = filename.find(".");
        std::string name = filename.substr(0, dot) + "_" + suffix + ".png";
        cv::imwrite("../mazes/" + name, out);
        display("Solution", out);
    }
// protected:
    cv::Mat im, out;
    std::string filename;
    std::vector<cv::Point> path;
    std::vector<std::string> windows;
};
}


#endif

