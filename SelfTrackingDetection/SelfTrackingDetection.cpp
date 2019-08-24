#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;
int LowH = 0, LowS = 0, LowV = 0, HighH = 150, HighS = 150, HighV = 150;
Ptr<Tracker> tracker = TrackerKCF::create();

class Main {

int main()
{
	VideoCapture Video(0);
	Mat VideoFrame;
	Mat VideoFrameThresholded;
	Mat VideoTracked;
	namedWindow("Control", WINDOW_AUTOSIZE);
	createTrackbar("LowH", "Control", &LowH, 255);
	createTrackbar("LowS", "Control", &LowS, 255);
	createTrackbar("LowV", "Control", &LowV, 255);
	createTrackbar("HighH", "Control", &HighH, 255);
	createTrackbar("HighS", "Control", &HighS, 255);
	createTrackbar("HighV", "Control", &HighV, 255);
	Rect2d bbox(100, 100, 100, 100);
	tracker->init(VideoTracked, bbox);
	Video.read(VideoFrame);
	VideoFrameThresholded = VideoFrame.clone();
	VideoTracked = VideoFrame.clone();
	imshow("Orginial", VideoFrame);
	imshow("Thresholded", VideoFrameThresholded);
	imshow("VideoTracked", VideoTracked);

	
	while (true)
	{
		Video.read(VideoFrame);
		inRange(VideoFrame, Scalar(LowH, LowS, LowV), Scalar(HighH, HighS, HighV), VideoFrameThresholded);
		erode(VideoFrameThresholded, VideoFrameThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(VideoFrameThresholded, VideoFrameThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(VideoFrameThresholded, VideoFrameThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(VideoFrameThresholded, VideoFrameThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		VideoTracked = VideoFrame.clone();
		VideoTracked.setTo(Scalar(0, 0, 0), ~VideoFrameThresholded);
		VideoTracked.setTo(Scalar(255, 255, 255), VideoFrameThresholded);
		imshow("Orginial", VideoFrame);
		imshow("Thresholded", VideoFrameThresholded);
		imshow("VideoTracked", VideoTracked);


		if (waitKey(1) == 27)
		{
			cout << "esc pressed";
			break;
		}
		if (waitKey(1) == 116)
		{
			tracking(VideoFrame, VideoFrameThresholded, VideoTracked, bbox);
		}
		
	}
	
}

int tracking(Mat VideoFrame, Mat VideoFrameThresholded, Mat VideoTracked, Rect2d bbox)
{
	while (true)
	{
		tracker->update(VideoTracked, bbox);
		rectangle(VideoTracked, bbox, Scalar(255, 0, 0), 2, 1);
		imshow("Orginial", VideoFrame);
		imshow("Thresholded", VideoFrameThresholded);
		imshow("VideoTracked", VideoTracked);
		if (waitKey(1) == 27)
		{
			cout << "esc pressed";
			break;
		}
	}
	return 0;
}
};