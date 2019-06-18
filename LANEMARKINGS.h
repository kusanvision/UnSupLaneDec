#include<iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

class LANEMARKINGS
{
    public:
        vector<Point> firstends;
		vector<Point> centers;
        vector<Point> secondends;
		vector<double> gradients;
        vector<Point> left;
        vector<Point> right;
};
