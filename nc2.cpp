#include<iostream>
#include<vector>
#include "opencv2/core/core.hpp"

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
    vector<cv::Point> pointlist;
    vector<int> list;
    for(int i = 1; i <= 5; i++)
        list.push_back(i);
    for(int j = list.size()-1; j >= 1; j--)
    {
        for(int k = list.size()-2; k >= 0; k--)
        {
            //std::cout << list[j] << "  " << list[k] << "\n";
            pointlist.push_back(cv::Point(list[j], list[k]));
        }
        list.resize(list.size()-1);
    }
    std::cout << "Size of pointlist = " << pointlist.size() << endl;
    for(int i = 0; i < pointlist.size(); i++)
        std::cout << pointlist[i];
    std::cout << "\n";
    return 0;
}

