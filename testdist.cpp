#include<iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "DISTANCE.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    cv::Point pt = Point(0,0);
    cv::Point pt1 = Point(-4,4);
    cv::Point pt2 = Point(4,4);

    double dist = abs(distance(pt, pt1, pt2));
    std::cout << "Distance from " << pt << " to line joining " << pt1 << " and " << pt2 << " is " << dist << endl;
    return 0;
}
