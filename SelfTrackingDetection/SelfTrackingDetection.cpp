#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/imgproc.hpp>
using namespace std;
using namespace cv;
int LowH = 0, LowS = 0, LowV = 7, HighH = 179, HighS = 255, HighV = 73, mX = 0, mY = 0, boxwidth = 0, boxheight = 0;
vector<vector<Point>> _contours;
vector<vector<Point>> squares; 
vector<Point> approx;
Point findHSVPoint(mX, mY);
TrackerKCF::MODE(GREY);
Ptr<Tracker> tracker = TrackerKCF::create();
vector<Vec4i> hierarchy;
Rect2d trackingbox;
Mat VideoFrame;
Mat VideoFrameThresholded;
Mat VideoTracked;
Mat VideoHSV;
Rect2d bbox;
bool spressed = false;


static double angle(Point pt1, Point pt2, Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1 * dy2) / sqrt((dx1*dx1 + dy1 * dy1)*(dx2*dx2 + dy2 * dy2) + 1e-10);
}
int run(bool tpressed, Mat VideoTracked, Rect2d bbox)
{
	if (tpressed == true)
	{
		tracker->init(VideoTracked, bbox);
	}
	return 0;
}

void findHSV(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		cout << "Left button of the mouse is clicked - position (" << mX << ", " << mY << ")" << endl;
		mX = x, mY = y;
		Vec3b color = VideoHSV.at<Vec3b>(Point(mX, mY));
		HighH = (color.val[0] + 100), HighS = (color.val[1] + 100), HighV = (color.val[2] + 100), LowH = (color.val[0] - 100), LowS = (color.val[1] - 100), LowV = (color.val[2] - 100);
	}
}

int main()
{
	VideoCapture Video(/*"D:/frc tracking test.mp4"*/ 0 );
	bool dpressed = false; 
	namedWindow("Control", WINDOW_AUTOSIZE);
	createTrackbar("LowH", "Control", &LowH, 179);
	createTrackbar("LowS", "Control", &LowS, 255);
	createTrackbar("LowV", "Control", &LowV, 255);
	createTrackbar("HighH", "Control", &HighH, 179);
	createTrackbar("HighS", "Control", &HighS, 255);
	createTrackbar("HighV", "Control", &HighV, 255);
	/*Rect2d bbox(Point (100, 100), Point (50, 50));*/
	Video.read(VideoFrame);
	VideoTracked = VideoFrame.clone();
	VideoFrameThresholded = VideoFrame.clone();
	cvtColor(VideoFrame, VideoHSV, COLOR_BGR2HSV, 0);
	imshow("Original", VideoFrame);
	imshow("Thresholded", VideoFrameThresholded);
	imshow("VideoTracked", VideoTracked);
	imshow("Video HSV", VideoHSV);
	bool tpressed = false;
	while (true)
	{
		Mat VideoContours = Mat::zeros(VideoFrame.size().height, VideoFrame.size().width, CV_8UC3);
		Mat VideoApprox = Mat::zeros(VideoFrame.size().height, VideoFrame.size().width, CV_8UC3);
		vector<vector<Point>> squares;
		Video.read(VideoFrame);
		cvtColor(VideoFrame, VideoHSV, COLOR_BGR2HSV, 0);
		VideoFrameThresholded = VideoFrame.clone();
		cvtColor(VideoFrameThresholded, VideoFrameThresholded, COLOR_BGR2HSV, 0);
		inRange(VideoFrameThresholded, Scalar(LowH, LowS, LowV), Scalar(HighH, HighS, HighV), VideoFrameThresholded);
		erode(VideoFrameThresholded, VideoFrameThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(VideoFrameThresholded, VideoFrameThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(VideoFrameThresholded, VideoFrameThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(VideoFrameThresholded, VideoFrameThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		//GaussianBlur(VideoFrameThresholded, VideoFrameThresholded, Size(3, 3), 0, 0, BORDER_CONSTANT);
		VideoTracked = VideoFrame.clone();
		findContours(VideoFrameThresholded, _contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
		drawContours(VideoContours, _contours, -1, Scalar(0, 255, 0), 3, LINE_AA, hierarchy, 3);
		for (size_t i = 0; i < _contours.size(); i++) {
			approxPolyDP(_contours[i], approx, arcLength(_contours[i], true)*0.02, true);
			if (approx.size() == 4 &&
				fabs(contourArea(approx)) > 1000 && isContourConvex(approx))
			{
				double maxCosine = 0;
				for (int j = 2; j < 5; j++)
				{
					// find the maximum cosine of the angle between joint edges
					double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
					maxCosine = MAX(maxCosine, cosine);
				}
				// if cosines of all angles are small
				// (all angles are ~90 degree) then write quandrange
				// vertices to resultant sequence
				if (maxCosine < 0.3)
					squares.push_back(approx);

			}
		}
		for (size_t i = 0; i < squares.size(); i++)
		{
			const Point* p = &squares[i][0];
			int n = (int)squares[i].size();
			polylines(VideoApprox, &p, &n, 1, true, Scalar(0, 255, 0), 1, LINE_AA);
		}
		//Point point((((approx[2].x - approx[1].x) / 2) + approx[1].x), (((approx[1].y - approx[3].y) / 2) + approx[3].y));
		//circle(VideoApprox, point, 3, Scalar(255, 0, 0), 3, 8, 0);
		VideoTracked.setTo(Scalar(0, 0, 0), ~VideoFrameThresholded);
		VideoTracked.setTo(Scalar(255, 255, 255), VideoFrameThresholded);
		if (waitKey(1) == 113)
		{
			boxwidth = (approx[1].x - approx[2].x);
			boxheight = (approx[1].y - approx[3].y);
		}
		if (waitKey(1) == 100)
		{
			dpressed = true;
			bbox = Rect2d(approx[1], approx[3]);
			cout << "drew box" << endl;
		}
		if (dpressed == true)
		{
			bbox = Rect2d(approx[1], approx[3]);
		}
		if (waitKey(1) == 116)
		{
			if (tpressed == false)
			{
				tpressed = true;
				run(tpressed, VideoTracked, bbox);
				cout << "Tracking Enabled" << endl;
			}
		}
		if (tpressed == true)
		{
			tracker->update(VideoTracked, bbox);
			rectangle(VideoTracked, bbox, Scalar(255, 0, 0), 2, 1);
		}
		if (waitKey(1) == 115)
		{
			spressed = true;
		}
		setMouseCallback("Video HSV", findHSV, NULL);
		

		imshow("Original", VideoFrame);
		imshow("Video HSV", VideoHSV);
		imshow("Thresholded", VideoFrameThresholded);
		imshow("VideoTracked", VideoTracked);
		imshow("VideoContours", VideoContours);
		imshow("VideoApprox", VideoApprox);
		if (waitKey(1) == 27)
		{
			cout << "esc pressed";
			break;
		}
	}
}



