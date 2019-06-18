#include<iostream>
#include<vector>
#include "opencv2/core/core.hpp"
#include "LANEMARKINGS.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    cv::Point pt1 = Point(10, 10);
    cv::Point pt2 = Point(20, 20);
    LANEMARKINGS lm;
    lm.centers.push_back(pt1);
    lm.centers.push_back(pt2);

    double grad1 = 1.0;
    double grad2 = 0.6;
    lm.gradients.push_back(grad1);
    lm.gradients.push_back(grad2);

    std::cout << "lm.centers[0] = " << lm.centers[0] << " lm.gradients[0] = " << lm.gradients[0] << "\n";
    std::cout << "lm.centers[1] = " << lm.centers[1] << " lm.gradients[1] = " << lm.gradients[1] << "\n";
    lm.LANEMARKINGS();
    std::cout << "lm.firstends[0] = " << lm.firstends[0] << "\n";
    //std::cout << "lm.firstends[1] = " << lm.firstends[1] << "\n";
    //std::cout << "lm.secondends[0] = " << lm.secondends[0] << "\n";
    //std::cout << "lm.secondends[1] = " << lm.secondends[1] << "\n";
    return 0;
}
