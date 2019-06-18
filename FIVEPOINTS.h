#include<iostream>
#include "opencv2/core/core.hpp"
#include "DISTANCE.h"

using namespace std;
using namespace cv;

void FIVEPOINTS(vector<vector<Point> > &contours, vector<cv::Point> points)
{
	vector<Point2d> eigen_vecs(2);
	vector<double> eigen_val(2);
    vector<Point> ends;
	double grad;
	
	for(int i = 0; i < contours.size(); ++i)
	{
		cv::Mat data_pts = cv::Mat(contours[i].size(), 2, CV_64FC1);
		for(int j = 0; j < data_pts.rows; ++j)
		{
			data_pts.at<double>(j, 0) = contours[i][j].x;
			data_pts.at<double>(j, 1) = contours[i][j].y;
		}
		PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
		Point center = Point(pca_analysis.mean.at<double>(0,0), pca_analysis.mean.at<double>(0,1));
		for(int k = 0; k < 2; ++k)
		{
			eigen_vecs[k] = Point2d(pca_analysis.eigenvectors.at<double>(k,0), pca_analysis.eigenvectors.at<double>(k,1));
			eigen_val[k] = pca_analysis.eigenvalues.at<double>(0,k);
		}
    /* 
     *
     *
     */
	}
}

void findends(cv::Point &center, double &grad, vector<Point> &ends)
{
    int c = 0;
    int fx, fy, sx, sy;
    c = center.y - grad*center.x;
    fx = center.x - 3;
    fy = grad*fx + c;
    sx = center.x + 3;
    sy = grad*sx + c;
    cv::Point pt1 = Point(fx, fy);
    cv::Point pt2 = Point(sx, sy);
    ends.push_back(pt1);
    ends.push_back(pt2);
}

void getOrientation(vector<Point> &pts, Mat &img)
{
    //if (pts.size() == 0) return false;

    Mat data_pts = Mat(pts.size(), 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i)
    {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }
    
    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
    Point pos = Point(pca_analysis.mean.at<double>(0, 0), pca_analysis.mean.at<double>(0, 1));

    //Store the eigenvalues and eigenvectors
    vector<Point2d> eigen_vecs(2);
    vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i)
    {
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                pca_analysis.eigenvectors.at<double>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }

    circle(img, pos, 2, CV_RGB(255, 0, 255), 0);
    if(atan2(eigen_vecs[0].y, eigen_vecs[0].x) > 0) 
    {
        line(img, pos - 0.1 * Point(eigen_vecs[0].x * eigen_val[0], eigen_vecs[0].y * eigen_val[0]), pos + 0.1 * Point(eigen_vecs[0].x * eigen_val[0], eigen_vecs[0].y * eigen_val[0]) , CV_RGB(0, 0, 235), 2, CV_AA);
    }
    else
        line(img, pos - 0.1*Point(eigen_vecs[0].x * eigen_val[0], eigen_vecs[0].y * eigen_val[0]), pos + 0.1 * Point(eigen_vecs[0].x * eigen_val[0], eigen_vecs[0].y * eigen_val[0]) , CV_RGB(240, 0, 0), 2, CV_AA);

    std::cout << "Angles = " << atan2(eigen_vecs[0].y, eigen_vecs[0].x) << endl;
    std::cout << endl;
}
