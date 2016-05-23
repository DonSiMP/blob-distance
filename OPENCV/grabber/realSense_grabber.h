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
#include "pxcsensemanager.h"
#include "pxcmetadata.h"
#include "pxcprojection.h"
#include <opencv2/core/mat.hpp>
#include <boost/thread.hpp>
#include <limits.h>

#ifdef F200_GRABBER
#define WIDTH	640
#define HEIGHT	480
#define FPS		60
#endif
#ifdef R200_GRABBER
#define WIDTH	480//628
#define HEIGHT	360//468
#define FPS		60
#endif

typedef unsigned short      UINT16;
typedef unsigned char       UINT8;

class RealSenseGrabber
{
public:
	RealSenseGrabber();
	~RealSenseGrabber();
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
	PXCSenseManager* pp = NULL;
	PXCCaptureManager* cm;
	PXCImage::ImageData data;
	std::vector<UINT16> depthBuffer;

	boost::thread thread;
	mutable boost::mutex mutex;
	bool running;
	bool dataAvailable;
};

RealSenseGrabber::RealSenseGrabber()
	: pp(nullptr)
	, depthWidth(WIDTH)
	, depthHeight(HEIGHT)
	, depthBuffer()
	, data()
	, running(false)
	, dataAvailable(false)
	, minZ(0.)
	, maxZ(0.)
{
	// Create Sensor Instance
	pp = PXCSenseManager::CreateInstance();
	if (!pp) {
		throw std::exception("Exception : CreateInstance()");
	}

	PXCCaptureManager *cm = pp->QueryCaptureManager();
	if (!cm) {
		throw std::exception("Exception : QueryCaptureManager()");
	}

	pp->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, WIDTH, HEIGHT, FPS);
	pxcStatus sts = pp->Init();
	if (sts < PXC_STATUS_NO_ERROR) {
		throw std::exception("Exception : QueryCaptureManager()");
	}

	PXCCapture::Device *device = cm->QueryDevice();

	// To Reserve Depth Frame Buffer
	depthBuffer.resize(depthWidth * depthHeight);
}

RealSenseGrabber::~RealSenseGrabber() throw()
{
	stop();
	thread.join();

	// End Processing
	if (pp) {
		pp->Close();
		pp->Release();
	}
}

void RealSenseGrabber::start() {
	running = true;

	thread = boost::thread(&RealSenseGrabber::threadFunction, this);
}

bool RealSenseGrabber::isDataAvailable()
{
	return dataAvailable;
}

std::vector<UINT16> * RealSenseGrabber::getDepthData()
{
	return &depthBuffer;
}

void RealSenseGrabber::stop()
{
	boost::unique_lock<boost::mutex> lock(mutex);
	running = false;
	lock.unlock();
}

size_t RealSenseGrabber::getFrameSize()
{
	return size_t(depthBuffer.size());
}

void RealSenseGrabber::threadFunction()
{
	while (running)
	{
		boost::unique_lock<boost::mutex> lock(mutex);
		pxcStatus sts = pp->AcquireFrame(true);
		if (sts < PXC_STATUS_NO_ERROR) {
			throw std::exception("Exception : pp::AcquireFrame()");
		}
		const PXCCapture::Sample *sample = pp->QuerySample();

		if (sample) {
			sts = sample->depth->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH, &data);
			if (sts >= PXC_STATUS_NO_ERROR) {
				memcpy(&depthBuffer[0], data.planes[0], sizeof(UINT16) * WIDTH * HEIGHT);
				dataAvailable = true;
			}
			sample->depth->ReleaseAccess(&data);
		}
		pp->ReleaseFrame();
		lock.unlock();
	}
}

double RealSenseGrabber::getRelativeValue(double depth) {
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

bool RealSenseGrabber::fillFrame(cv::Mat * destData)
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
		return true;
	}
	return false;
}