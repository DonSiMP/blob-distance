/*
Copyright 2016 Chris Papenfu�

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
#include <stdio.h>
#include <Windows.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#if defined(F200_GRABBER) || defined(R200_GRABBER)
#include "grabber\realSense_grabber.h"
#define CAM_HEIGHT 226.0
#endif // F200_GRABBER
#ifdef KINECT2_GRABBER
#include "grabber\kinect_grabber.h"
#define CAM_HEIGHT 400.0
#endif // KINECT2_GRABBER
#ifdef DUO_GRABBER
#include "grabber\duo_grabber.h"
#define CAM_HEIGHT 400.0
#endif // KINECT2_GRABBER
#ifdef LEAP_GRABBER
#include "grabber\leap_grabber.h"
#define CAM_HEIGHT 210.0
#endif // LEAP_GRABBER


/*KONFIG VALUES in mm*/
/*END KONFIG VALUES*/
const double sqCamHeight = CAM_HEIGHT * CAM_HEIGHT;
/*END CONST VALUES FROM KONFIG*/



using namespace std;

void detectAndDraw(cv::Mat& img, cv::CascadeClassifier& cascade,
	cv::CascadeClassifier& nestedCascade,
	double scale, bool tryflip, int frameNum);

string cascadeName;
string nestedCascadeName;
int idx = 0;

bool buildBackground = true;
cv::Mat fgMask, bgMask; //fg mask fg mask generated by MOG2 method
cv::Ptr<cv::BackgroundSubtractor> bgSub; //MOG2 Background subtractor

cv::Ptr<cv::SimpleBlobDetector> blobber;
std::vector<cv::KeyPoint> keypoints;

int main(int, char**) {
	int frameNum = 0;
	double scale;
	bool tryflip;
	cv::CascadeClassifier cascade, nestedCascade;
	//bgSub = cv::createBackgroundSubtractorKNN(90);
	bgSub = cv::createBackgroundSubtractorMOG2(100, 16.0, false); //MOG2 approach

	cv::SimpleBlobDetector::Params params = cv::SimpleBlobDetector::Params();
	params.filterByArea = true;
	params.minArea = 100;
	params.maxArea = 100000;
	params.filterByColor = false;
	//params.filterByCircularity = true;
	//params.minCircularity = 0.6;
	//params.maxCircularity = 0.9;
	params.filterByConvexity = false;
	params.filterByInertia = false;
	blobber = cv::SimpleBlobDetector::create(params);

	scale = 0.5;
	tryflip = false;

	//open the video stream and make sure it's opened
#ifdef KINECT2_GRABBER
	KinectGrabber
#endif // KINECT2_GRABBER
#ifdef DUO_GRABBER
		DuoGrabber
#endif // DUO_GRABBER
#ifdef LEAP_GRABBER
		LeapGrabber
#endif // LEAP_GRABBER
#if defined(F200_GRABBER) || defined(R200_GRABBER)
		RealSenseGrabber
#endif // F200_GRABBER
		grabber;
	grabber.start();
	cv::Mat * frame = new cv::Mat(grabber.depthHeight, grabber.depthWidth, CV_8UC1);
	grabber.minZ = 700.;
	grabber.maxZ = 1300.;// 1500.;
#ifdef F200_GRABBER
	grabber.maxZ = 500.;// 1500.;
#endif // F200_GRABBER
#ifdef FR00_GRABBER
	grabber.maxZ = 1000.;// 1500.;
#endif // R200_GRABBER

	//frame->resize(grabber.getFrameSize());
	size_t idx = 0;
	for (;;) {
		try
		{
			if (!grabber.fillFrame(frame)) {
				continue;
			}
			if (buildBackground) {
				bgSub->apply(*frame, fgMask, .8);
			}
			else {
				bgSub->apply(*frame, fgMask, 0.);
			}

			cv::Mat fgFrame, fgFrameBlobs;
			frame->copyTo(fgFrame, fgMask);
			if (!buildBackground) {
				blobber->detect(fgFrame, keypoints);//find Blobs in image (background substracted)
				for each (cv::KeyPoint var in keypoints)
				{
					//calculate middle distance at keypoint
					double dist = 0., cnt = 0.;
					for (size_t x = var.pt.x - 10; x < var.pt.x + 10 && x < grabber.depthWidth; x++)
					{
						for (size_t y = var.pt.y - 10; y < var.pt.y + 10 && y < grabber.depthHeight; y++)
						{
							size_t idx = y * grabber.depthWidth + x;
							UINT16 val = (*grabber.getDepthData())[idx];
							if (val > 0) {
								dist += val;
								cnt++;
							}
						}
					}
					dist /= cnt;//middl Value
					/*##################### calculated distance #####################*/
					dist = sqrt(dist*dist - sqCamHeight);//real distance using pytagoras
					/*##################### calculated distance #####################*/
					cv::putText(fgFrame, to_string(dist) + " mm", var.pt, cv::FONT_HERSHEY_PLAIN, 3., cv::Scalar(255, 255, 0));
				}
				cv::drawKeypoints(fgFrame, keypoints, fgFrameBlobs, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
				cv::imshow("Foreground with Blobs", fgFrameBlobs);
			}
			cv::imshow("result", *frame);

			imshow("Foreground Frame", fgFrame);
			int c = cv::waitKey(10);
			if (c == 27 || c == 'q' || c == 'Q') {
				break;
			}
			if (c == 'b' || c == 'B') {
				buildBackground = !buildBackground;
				if (buildBackground) {
					bgSub->clear();
				}
			}
		}
		catch (const std::exception&)
		{

		}
	}
	grabber.stop();
}

