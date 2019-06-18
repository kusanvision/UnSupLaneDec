#include<iostream>
#include "opencv2/core/core.hpp"

using namespace std;
using namespace cv;

int main()
{
    double d1 = 4.9;
    double d2 = 6.3;
    cv::Point pt = Point(d1, d2);
    int i = d1;
    std::cout << d1 << "\n";
    std::cout << i << "\n";
    std::cout << pt;
    return 0;
}
