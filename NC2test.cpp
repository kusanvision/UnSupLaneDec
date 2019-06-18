#include<iostream>
#include<vector>
#include "opencv2/core/core.hpp"

using namespace std;
using namespace cv;
void NC2(int, vector<Point> &);

int main(int argc, char **argv)
{
    vector<cv::Point> pointlist;
    int size = 5;
    NC2(size, pointlist);
    for(int i = 0; i < pointlist.size(); i++)
        std::cout << pointlist[i];
    return 0;
}

void NC2(int num, vector<Point> &pointlist)
{
    vector<int> list;
    for(int i = 1; i <= num; i++)
        list.push_back(i);

    for(int j = list.size()-1; j >= 1; --j)
    {
        for(int k = list.size()-2; k >= 0; --k)
            pointlist.push_back(cv::Point(list[j], list[k]));
        list.resize(list.size()-1);
    }
}
