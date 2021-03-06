#include <opencv2/opencv.hpp>
#include <iostream>
#include <queue>
#include <Windows.h>
#include "UnipathMatch\GeometricRecognizer.h"
using namespace cv;
using namespace std;

void text(Mat frame, String text, Point loc = Point(10, 20), Scalar color = Scalar(255, 0, 0)) {
	putText(frame, text, loc, FONT_HERSHEY_SIMPLEX, 1, color, 2);
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

	//calculate size of calibration rectangle
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
		rectangle(frame, rectStart - border, rectEnd + border, Scalar(255));
		text(frame, "Filter calibration");

		imshow(imgWindow, frame);
		if (waitKey(30) >= 0) break;
	}
	return;
}

const char* minContourTrackbar = "Min area";

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

		//find largest area contour
		vector<double> areas;
		double maxi = -1;
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
			if (areas[i] > minContourArea) //filter out too small contours
				drawContours(frame, contours, i, i == maxidx ? Scalar(0, 0, 255) : Scalar(255, 0, 0), 2);

		text(frame, "Contour calibration");

		imshow(imgWindow, frame);
		if (waitKey(30) >= 0) break;
	}
}

void initProcess();
void process(Point center, Point last, Mat frame);

const char* tickPeriodTrackbar = "Tick time";
const int64 tickMult = 10000;
const Point none(-1, -1);

int tickPeriod = 55;
void run() {
	createTrackbar(tickPeriodTrackbar, controlWindow, &tickPeriod, getTickFrequency() / tickMult);
	initProcess();

	queue<pair<int64, Point>> prevPoints;

	int64 lastFrameTick = 0;

	while (true) {
		int64 currTime = getTickCount();
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
			center = Point((int)(m.m10 / m.m00), (int)(m.m01 / m.m00));
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

		//Display FPS
		text(frame, "FPS: " + to_string(1 / ((currTime - lastFrameTick) / getTickFrequency())), Point(10, frameHeight));
		lastFrameTick = currTime;

		imshow(imgWindow, frame);
		if (waitKey(30) >= 0) break;
	}
}

const char* minXSpeedTrackbar = "Min dx/t";
const char* minYSpeedTrackbar = "Min dy/t";
const char* debounceTrackbar = "Debounce";

int minXSpeed = 160;
int minYSpeed = 100;
int debounceTick = 160;
void initProcess() {
	createTrackbar(minXSpeedTrackbar, controlWindow, &minXSpeed, frameHeight + frameWidth);
	createTrackbar(minYSpeedTrackbar, controlWindow, &minYSpeed, frameHeight + frameWidth);
	createTrackbar(debounceTrackbar, controlWindow, &debounceTick, (int)getTickFrequency() / tickMult);
}

void(*actionCallback)(int, Mat);

int lastAction = -1;
int64 lastActionTick = 0;
void process(Point center, Point last, Mat frame) {
	if (center != none && last != none) {
		int dx = center.x - last.x;
		int dy = center.y - last.y;

		//check movement thresholds
		int action = -1;
		if (dy > minYSpeed) {
			action = 1;
		}
		else if (dy < -minYSpeed) {
			action = 2;
		}

		if (dx > minXSpeed) {
			action = 3;
		}
		else if (dx < -minXSpeed) {
			action = 4;
		}

		if (action != -1) {
			//debounce
			int64 currTick = getTickCount();
			if (currTick - lastActionTick > debounceTick * tickMult) {
				actionCallback(action, frame);
				lastActionTick = currTick;
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

void sendKey(WORD vkey) {
	INPUT input;
	input.type = INPUT_KEYBOARD;
	input.ki.wScan = MapVirtualKey(vkey, MAPVK_VK_TO_VSC);
	input.ki.time = 0;
	input.ki.dwExtraInfo = 0;
	input.ki.wVk = vkey;
	input.ki.dwFlags = 0; // there is no KEYEVENTF_KEYDOWN
	SendInput(1, &input, sizeof(INPUT));

	input.ki.dwFlags = KEYEVENTF_KEYUP;
	SendInput(1, &input, sizeof(INPUT));
}

void myActionCallback(int action, Mat frame) {
	if (action == 1) {
		text(frame, "Volume down");
		sendKey(VK_VOLUME_DOWN);
	}
	if (action == 2) {
		text(frame, "Volume up");
		sendKey(VK_VOLUME_UP);
	}
	if (action == 4) {
		text(frame, "Next");
		sendKey(VK_MEDIA_NEXT_TRACK);
	}
	if (action == 3) {
		text(frame, "Prev");
		sendKey(VK_MEDIA_PREV_TRACK);
	}
}

int main(int, char**)
{
	cvUseOptimized(true);

	actionCallback = myActionCallback;

	setNumThreads(0);//limit cpu usage

	cap = VideoCapture(0);
	if (!cap.isOpened())  // check if we succeeded
		return -1;

	frameWidth = (int)cap.get(CAP_PROP_FRAME_WIDTH);
	frameHeight = (int)cap.get(CAP_PROP_FRAME_HEIGHT);

	namedWindow(controlWindow, WINDOW_NORMAL);

	calibFilter();
	calibContour();
	run();

	cap.release();
}