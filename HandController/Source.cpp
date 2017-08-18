#include <opencv2/opencv.hpp>
#include <iostream>
#include <queue>
using namespace cv;
using namespace std;

void text(Mat frame, String text, Point loc = Point(10, 20), Scalar color = Scalar(255, 0, 0)) {
	putText(frame, text, loc, FONT_HERSHEY_SIMPLEX, 1, color);
}

const char* controlWindow = "Control window";
const char* imgWindow = "Image";

VideoCapture cap; // open the default camera
int frameWidth, frameHeight;

const char* hueRangeTrackbar = "Hue Range";
const char* satRangeTrackbar = "Sat Range";
const char* valRangeTrackbar = "Val Range";

Scalar avg;
Scalar range;
void calibFilter() {
	int hueRange = 9,
		satRange = 75,
		valRange = 100;
	createTrackbar(hueRangeTrackbar, controlWindow, &hueRange, 256);
	createTrackbar(satRangeTrackbar, controlWindow, &satRange, 256);
	createTrackbar(valRangeTrackbar, controlWindow, &valRange, 256);

	//calibration loop
	int rectWidth = 100,
		rectHeight = 100;
	Point rectStart(frameWidth / 2 - rectWidth / 2, frameHeight / 2 - rectHeight / 2);
	Point rectEnd = rectStart + Point(rectWidth, rectHeight);
	Point border(1, 1);

	while (true) {
		Mat frame;
		cap >> frame;

		cvtColor(frame, frame, CV_BGR2HSV);

		Mat calib = frame(Rect(rectStart, rectEnd));

		avg = mean(calib);

		range = Scalar(hueRange, satRange, valRange);
		inRange(frame, avg - range, avg + range, frame);

		//draw overlay
		rectangle(frame, rectStart - border, rectEnd + border, Scalar(255, 0, 0));
		text(frame, "Filter calibration");

		imshow(imgWindow, frame);
		if (waitKey(30) >= 0) break;
	}
	return;
}

const char* minContourTrackbar = "Min contour area";

int minContourArea = 1000;
void calibContour() {
	createTrackbar(minContourTrackbar, controlWindow, &minContourArea, frameHeight * frameWidth / 4);

	while (true) {
		Mat frame;
		cap >> frame;

		Mat buffer;
		cvtColor(frame, buffer, CV_BGR2HSV);

		inRange(buffer, avg - range, avg + range, buffer);

		vector<vector<Point>> contours;
		findContours(buffer, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

		vector<int> areas;
		int maxi = -1;
		int maxidx = -1;
		for (int i = 0; i < contours.size(); i++) {
			areas.push_back(contourArea(contours[i]));
			if (areas[i] > maxi) {
				maxi = areas[i];
				maxidx = i;
			}
		}

		//draw overlay
		for (int i = 0; i < contours.size(); i++)
			if (areas[i] > minContourArea)
				drawContours(frame, contours, i, i == maxidx ? Scalar(0, 0, 255) : Scalar(255, 0, 0), 2);

		text(frame, "Contour calibration");

		imshow(imgWindow, frame);
		if (waitKey(30) >= 0) break;
	}
}

void initProcess();
void process(Point center, Point last, Mat frame);

const char* tickPeriodTrackbar = "Tick period";
const int64 tickMult = 10000;
const Point none(-1, -1);

int tickPeriod = 55;
void run() {
	createTrackbar(tickPeriodTrackbar, controlWindow, &tickPeriod, getTickFrequency() / tickMult);
	initProcess();

	queue<pair<int64, Point>> prevPoints;

	while (true) {
		//capture frame
		Mat frame;
		cap >> frame;

		Mat buffer;
		cvtColor(frame, buffer, CV_BGR2HSV);//convert to hsv

		inRange(buffer, avg - range, avg + range, buffer);//filter out desired color

		vector<vector<Point>> contours;
		findContours(buffer, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

		//find biggest contour, filter out stuff thats too small
		int maxi = -1;
		int maxIdx = -1;
		for (int i = 0; i < contours.size(); i++) {
			int alt = contourArea(contours[i]);
			if (alt < minContourArea)continue;
			if (alt > maxi) {
				maxi = alt;
				maxIdx = i;
			}
		}

		//calculate center of mass of tracked
		Point center = none;
		if (maxIdx != -1) {
			Moments m = moments(contours[maxIdx]);
			center = Point(m.m10 / m.m00, m.m01 / m.m00);
			prevPoints.push({ getTickCount(), center });
		}

		//pop out of date locations
		Point last = none;
		while (!prevPoints.empty()) {
			auto curr = prevPoints.front();
			if (curr.first > getTickCount() - tickMult*tickPeriod) {
				last = curr.second;
				break;
			}
			prevPoints.pop();
		}

		process(center, last, frame);

		imshow(imgWindow, frame);
		if (waitKey(30) >= 0) break;
	}
}

const char* minSpeedTrackbar = "Min px/tick";
const char* debounceTrackbar = "Debounce ticks";

int minSpeed = 160;
int debounceTick = 60;
void initProcess() {
	createTrackbar(minSpeedTrackbar, controlWindow, &minSpeed, frameHeight + frameWidth);
	createTrackbar(debounceTrackbar, controlWindow, &debounceTick, getTickFrequency() / tickMult);
}

void(*actionCallback)(int, Mat);

int lastAction = -1;
int64 lastActionTick = 0;
void process(Point center, Point last, Mat frame) {
	if (center != none && last != none) {
		int dx = center.x - last.x;
		int dy = center.y - last.y;

		int action = -1;
		if (dy > minSpeed) {
			action = 1;
		}
		else if (dy < -minSpeed) {
			action = 2;
		}

		if (dx > minSpeed) {
			action = 3;
		}
		else if (dx < -minSpeed) {
			action = 4;
		}

		if (action != -1) {
			if (lastAction == action) {
				int64 currTick = getTickCount();
				if (currTick - lastActionTick > debounceTick*tickMult) {
					actionCallback(action, frame);
					lastActionTick = currTick;
					lastAction = action;
				}
			}
			else {
				actionCallback(action, frame);
				lastAction = action;
			}
		}
	}

	//draw overlay
	if (center != none) {
		circle(frame, center, 2, Scalar(0, 0, 255), 2);
	}
	if (last != none) {
		circle(frame, last, 2, Scalar(0, 255, 0), 2);
	}
}

void myActionCallback(int aciton, Mat frame) {
	text(frame, "Action: " + to_string(aciton));
}

int main(int, char**)
{
	actionCallback = myActionCallback;

	cap = VideoCapture(0);
	if (!cap.isOpened())  // check if we succeeded
		return -1;

	frameWidth = cap.get(CAP_PROP_FRAME_WIDTH),
		frameHeight = cap.get(CAP_PROP_FRAME_HEIGHT);

	namedWindow(controlWindow);

	calibFilter();
	calibContour();
	run();
}