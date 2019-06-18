#include<iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "RANSAC.h"

using namespace std;
using namespace cv;

void getOrientation(vector<Point> &, cv::Mat &);
void leftYAright(vector<vector<Point> > &, LANEMARKINGS &, LANEMARKINGS &);
void findends(cv::Point &center, double &grad, vector<Point> &ends);

class FRAMEOBJECT
{
    public:
        FRAMEOBJECT(cv::Mat &frame);
        void PROCESS(void);
        void PREPROCESS(void);
    private:
        cv::Mat image, grayimage, thresh_op;
        LANEMARKINGS leftlanes;
        LANEMARKINGS rightlanes;
        vector<vector<Point> > contours;
        vector<vector<Point> > goodcontours;
        vector<Vec4i> hierarchy;
        vector<Point> leftends;
        vector<Point> rightends;
};

FRAMEOBJECT::FRAMEOBJECT(cv::Mat &frame)
{
    image = frame;
}
void FRAMEOBJECT::PREPROCESS(void)
{
    cv::Mat grad_x, grad_y, abs_grad_x, abs_grad_y, edgemap;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;
    if(image.channels() == 3)
        cvtColor(image, grayimage, CV_BGR2GRAY);
    else
        image.copyTo(grayimage);
    GaussianBlur(grayimage, grayimage, Size(3, 3), 0, 0,BORDER_DEFAULT);
    /*
    Sobel(grayimage, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
    convertScaleAbs(grad_x, abs_grad_x);
    Sobel(grayimage, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
    convertScaleAbs(grad_y, abs_grad_y);
    addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, edgemap);
    */
    threshold(grayimage, thresh_op, 175, 255, THRESH_BINARY);
    imshow("After threshold", thresh_op);
}
void FRAMEOBJECT::PROCESS(void)
{
    cv::findContours(thresh_op, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    for(int i = 0; i < contours.size(); ++i)
    {
        double area = contourArea(contours[i]);
        if(area > 1e5 || area < 1e1)
            continue;
        else
            goodcontours.push_back(contours[i]);
    }
    for(int i = 0; i < goodcontours.size(); ++i)
    {
        drawContours(image, goodcontours, -1, CV_RGB(0,0,0), 1, 8, hierarchy, CV_AA);
        //getOrientation(goodcontours[i], image);
    }
    leftYAright(goodcontours, leftlanes, rightlanes);
    int numofcontours = goodcontours.size();
    drawContours(image, goodcontours, -1, CV_RGB(0,0,0), 1, 8, hierarchy);
    //RANSAC(leftlanes, leftends);
    //RANSAC(rightlanes, rightends);
    //line(image, leftends[0], leftends[1], CV_RGB(0,0,240), 2, CV_AA);
    //line(image, rightends[0], rightends[1], CV_RGB(240,0,0), 2, CV_AA);
    std::cout << numofcontours << " are there in the frame" << endl;
    std::cout << "In LHS = " << leftlanes.centers.size() << "   In RHS = " << rightlanes.centers.size() << "\n";
}


////////////////////////////// MAIN ////////////////////////////////
int main(int argc, char **argv)
{
    if(argc != 2)
    {
        std::cout << "Wrong invocation" << endl;
        return -1;
    }
    std::string videofile = argv[1];
    cv::VideoCapture video;
    if(!video.open(videofile))
    {
        std::cout << "Can not open video file" << endl;
        return -2;
    }

    cv::Mat InputImage;
    int frame = 0;
    for(;;)
    {
        frame++;
        std::cout << "Frame number = " << frame << "\n";
        video >> InputImage;
        if(InputImage.empty())
            break;

        cv::Mat CroppedInput = InputImage(Range(210,360), Range::all());
        FRAMEOBJECT f = FRAMEOBJECT(CroppedInput);
        f.PREPROCESS();
        f.PROCESS();
        imshow("Detected lines", CroppedInput);
        waitKey(0);
    }
    return 0;
}

void leftYAright(vector<vector<Point> > &contours, LANEMARKINGS &left, LANEMARKINGS &right)
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
		grad = atan2(eigen_vecs[0].y, eigen_vecs[0].x);
		if(grad > 0)
		{
			right.centers.push_back(center);
			right.gradients.push_back(grad);
            findends(center, grad, ends);
            right.firstends.push_back(ends[0]);
            right.secondends.push_back(ends[1]);
		}
		else
		{
			left.centers.push_back(center);
			left.gradients.push_back(grad);
            findends(center, grad, ends);
            left.firstends.push_back(ends[0]);
            left.secondends.push_back(ends[1]);
		}
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
