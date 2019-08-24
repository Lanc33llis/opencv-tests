#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/ximgproc.hpp"

using namespace cv;
using namespace std;
using namespace ximgproc;
using namespace segmentation;

int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV;

int main()
{
	namedWindow("Control");
	VideoCapture Video(0); //captures the video from web cam
	Mat VideoFrameRead;
	Mat VideoThresholded;
	Mat VideoTracked;

	while (true)
	{
		if (waitKey(0) == 43)
		{
			iHighH++;
			iHighS++;
			iHighV++;
		}
		if (waitKey(0) == 45)
		{
			iHighH--;
			iHighS--;
			iHighV--;
		}
		int numShowRects = 5;
		Video >> VideoFrameRead;
		cvtColor(VideoFrameRead, VideoThresholded, COLOR_RGB2HSV);
		inRange(VideoThresholded, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), VideoThresholded);
		erode(VideoThresholded, VideoThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(VideoThresholded, VideoThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(VideoThresholded, VideoThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(VideoThresholded, VideoThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		VideoFrameRead.setTo(Scalar(0, 0, 0), ~VideoThresholded);
		VideoFrameRead.setTo(Scalar(255, 255, 255), VideoThresholded);
		VideoTracked = VideoFrameRead.clone();
		Ptr<SelectiveSearchSegmentation> sss = createSelectiveSearchSegmentation();
		sss->setBaseImage(VideoTracked);
		sss->switchToSelectiveSearchFast();
		std::vector<Rect> rects;
		sss->process(rects);
		for (int i = 0; i < rects.size(); i++) {
			if (i < numShowRects) {
				rectangle(VideoTracked, rects[i], Scalar(0, 255, 0));
			}
			else {
				break;
			}
		}

		imshow("Orginial", VideoFrameRead);
		imshow("Thresholded", VideoThresholded);
		imshow("Tracked", VideoTracked);
		if (waitKey(30) == 27)
		{
			break;
		}
	}
	return 0;
}
