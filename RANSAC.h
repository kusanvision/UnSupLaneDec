#include<iostream>
#include<algorithm>
#include<vector>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "LANEMARKINGS.h"
#include "DISTANCE.h"

using namespace std;
using namespace cv;

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

void RANSAC(LANEMARKINGS &markings, vector<Point> &lineends)
{
    double c = 0;
    if(markings.centers.size() == 1)
    {
        c = markings.centers[0].y - (markings.gradients[0])*(markings.centers[0].x);
        cv::Point pt = Point((markings.centers[0].x - 4), markings.gradients[0]*(markings.centers[0].x - 4) + c);
        lineends.push_back(pt);
        pt = Point((markings.centers[0].x + 4), markings.gradients[0]*(markings.centers[0].x + 4) + c);
        lineends.push_back(pt);
    }

    if(markings.centers.size() == 2)
    {
        lineends.push_back(markings.centers[0]);
        lineends.push_back(markings.centers[1]);
    }

   if(markings.centers.size() >= 3)
    {
        int num = markings.centers.size();
        vector<Point> combinations;
        NC2(num, combinations);
        int index1, index2, savedindex1, savedindex2;
        int consensus = 0;
        int highest_consensus = 0;
        double threshold = 8.0;

        for(int l = 0; l < combinations.size(); l++)
        {
            index1 = combinations[l].x;
            index2 = combinations[l].y;
            for(int j = 0; j < markings.centers.size(); ++j)
            {
                std::cout << "index1 = " << index1 << " index2 = " << index2 << "\n";
                if(linedistance(markings.centers[j], markings.centers[index1], markings.centers[index2]) <= threshold)
                    consensus++;
                if(linedistance(markings.firstends[j], markings.centers[index1], markings.centers[index2]) <= threshold)
                    consensus++;
                if(linedistance(markings.secondends[j], markings.centers[index1], markings.centers[index2]) <= threshold)
                    consensus++;
            }
            if(consensus > highest_consensus)
            {
                highest_consensus = consensus;
                savedindex1 = index1;
                savedindex2 = index2;
            }
        }
        lineends.push_back(markings.centers[savedindex1]);
        lineends.push_back(markings.centers[savedindex2]);
    }
}

