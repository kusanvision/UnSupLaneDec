#include<iostream>
#include<math.h>
#include<stdlib.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

double linedistance(cv::Point &from, cv::Point &endA, cv::Point &endB)
{
    double A = endA.y - endB.y;
    double B = endB.x - endA.x;
    return abs((A*(from.x - endA.x) + B*(from.y - endA.y)) / sqrt(A*A + B*B)) ;
}
