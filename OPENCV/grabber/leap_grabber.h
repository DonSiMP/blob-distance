/*
Copyright 2016 Chris Papenfuß

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
#pragma once
#include "Leap.h"
#include <opencv2/core/mat.hpp>
#include <boost/thread.hpp>
#include <limits.h>

#define WIDTH 640
#define HEIGHT 240
#define FPS 30

static const int minLight = 60;
bool allSet[640] = { false };

typedef unsigned short      UINT16;
typedef unsigned char       UINT8;

class LeapGrabber
{
public:
	LeapGrabber();
	~LeapGrabber();
	bool fillFrame(cv::Mat * destData);
	bool isDataAvailable();
	std::vector<UINT16> * getDepthData();
	void start();
	void stop();
	size_t getFrameSize();
	double maxZ;
	double minZ;

	int depthWidth;
	int depthHeight;
protected:
	void threadFunction();
	double getRelativeValue(double depth);
	int findIdx(UINT8 searchVal, bool* idxs, const UINT8* image_buffer, const int start, const int end, const int treshold);
	//cv::Mat disp, disp8;
	//cv::Mat * l = new cv::Mat(cv::Size(WIDTH, HEIGHT), CV_8UC1), *r = new cv::Mat(cv::Size(WIDTH, HEIGHT), CV_8UC1);
	Leap::Controller controller;
	std::vector<UINT16> depthBuffer;

	boost::thread thread;
	mutable boost::mutex mutex;
	bool running;
	bool dataAvailable;
};

int LeapGrabber::findIdx(UINT8 searchVal, bool* idxs, const UINT8* image_buffer, const int start, const int end, const int treshold) {
	for (int col = start, idx = 0; col < end; col++, idx++)
	{
		if (idxs[idx])continue;
		UINT8 val = image_buffer[col];
		if (val < minLight) continue;//to dark
		if (val == searchVal || (val - treshold < searchVal && val + treshold>searchVal)) {
			idxs[idx] = true;
			return col - start;
		}
	}
	return -1;
}

LeapGrabber::LeapGrabber()
	: depthWidth(WIDTH)
	, depthHeight(HEIGHT)
	, depthBuffer()
	, running(false)
	, dataAvailable(false)
	, minZ(0.)
	, maxZ(0.)
{
	controller.setPolicy(Leap::Controller::POLICY_IMAGES);

	// To Reserve Depth Frame Buffer
	depthBuffer.resize(depthWidth * depthHeight);
}

LeapGrabber::~LeapGrabber() throw()
{
	stop();
	thread.join();

}

void LeapGrabber::start() {
	running = true;

	thread = boost::thread(&LeapGrabber::threadFunction, this);
}

bool LeapGrabber::isDataAvailable()
{
	return dataAvailable;
}

std::vector<UINT16> * LeapGrabber::getDepthData()
{
	return &depthBuffer;
}

void LeapGrabber::stop()
{
	boost::unique_lock<boost::mutex> lock(mutex);
	running = false;
	lock.unlock();
}

size_t LeapGrabber::getFrameSize()
{
	return size_t(depthBuffer.size());
}

void LeapGrabber::threadFunction()
{
	while (running)
	{
		boost::unique_lock<boost::mutex> lock(mutex);
		Leap::Frame frame = controller.frame();
		Leap::ImageList images = frame.images();
		if (!images[0].isValid() || !images[1].isValid())continue;
		const UINT8* image_bufferL = images[0].data(); // brightness values
		const UINT8* image_bufferR = images[1].data(); // brightness values

		//VERSUCH Ermittlung der Tiefeninfos mit openCV
		//std::memcpy(l->data, image_bufferL, sizeof(UINT8) * HEIGHT * WIDTH);
		//std::memcpy(r->data, image_bufferR, sizeof(UINT8) * HEIGHT * WIDTH);
		//cv::Ptr<cv::StereoMatcher> stereo = cv::StereoSGBM::create(0, 16, 21);
		////cv::StereoBM::create(numDisparities, blockSize);
		//stereo->compute(*l, *r, disp);
		//disp.convertTo(disp8, CV_8U);
		//VERSUCH ENDE

		lock.unlock();
		size_t validCnt = 0;
		UINT16 * depthPtr = &depthBuffer[0];
		for (int row = 0; row < HEIGHT; row++)
		{
			std::fill(allSet, allSet + 640, false);//remember which pixel already has its pair
			const int start = row * images[0].width() * images[0].bytesPerPixel();
			for (int col = start; col < start + images[0].width(); col++, depthPtr++)
			{
				UINT8 valL = image_bufferL[col];
				if (valL < minLight) {
					*depthPtr = 0;
					continue;//to dark
				}
				int colR = findIdx(valL, allSet, image_bufferR, start, start + images[0].width(), 20);
				if (colR > 0) {
					Leap::Vector slopes_left = images[0].rectify(Leap::Vector(col - start, row, 0));
					Leap::Vector slopes_right = images[1].rectify(Leap::Vector(colR, row, 0));
					float z = 40 / (slopes_right.x - slopes_left.x);
					if (z < 0.) {
						*depthPtr = 0;
					}
					else {
						*depthPtr = (UINT16)z;
						validCnt++;
						//std::cout << std::to_string(z) << std::endl;
					}

				}
				else {
					*depthPtr = 0;
				}
			}
		}
		if (validCnt > 0) {
			dataAvailable = true;
		}

	}
}

double LeapGrabber::getRelativeValue(double depth) {
	if (maxZ == 0.) {
		maxZ = 4000.;
	}
	if (maxZ > 0. && depth > maxZ) {
		depth = 0.;
	}
	if (minZ > 0. && minZ > depth) {
		depth = minZ;
	}
	depth = depth - minZ;
	if (maxZ > minZ) {
		depth = depth / (maxZ - minZ);
	}
	else {
		depth = depth / 4000.;
	}
	return depth;
}

bool LeapGrabber::fillFrame(cv::Mat * destData)
{
	if (dataAvailable) {
		dataAvailable = false;
		for (size_t i = 0; i < depthBuffer.size(); i++) {
			double depth = getRelativeValue(depthBuffer[i]);

			//depth = depth * USHRT_MAX;
			//destData->at<UINT16>(i) = depth;

			depth = depth * UCHAR_MAX;

			destData->at<UINT8>(i) = depth;
		}
		//cv::imshow("Leap-Left", *l);
		//cv::imshow("Right-Left", *r);
		//cv::imshow("Disparity", disp8);
		return true;
	}
	return false;
}