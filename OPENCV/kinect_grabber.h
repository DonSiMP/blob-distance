/*
Copyright 2016 Chris Papenfuﬂ

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
#include <Kinect.h>
#include <opencv2/core/mat.hpp>
#include <boost/thread.hpp>
#include <limits.h>

class KinectGrabber
{
public:
	KinectGrabber();
	~KinectGrabber();
	bool fillFrame(cv::Mat * destData);
	bool isDataAvailable();
	cv::Mat getMat();
	void start();
	void stop();
	size_t getFrameSize();
	double maxZ;
	double minZ;

	int depthWidth;
	int depthHeight;
protected:
	void threadFunction();

	HRESULT result;
	IKinectSensor* sensor;
	IDepthFrameSource* depthSource;
	IDepthFrameReader* depthReader;
	std::vector<UINT16> depthBuffer;

	boost::thread thread;
	mutable boost::mutex mutex;
	bool running;
	bool dataAvailable;
};

KinectGrabber::KinectGrabber()
	:sensor(nullptr)
	, depthSource(nullptr)
	, depthReader(nullptr)
	, result(S_OK)
	, depthWidth(512)
	, depthHeight(424)
	, depthBuffer()
	, running(false)
	, dataAvailable(false)
	, minZ(0.)
	, maxZ(0.)
{
	// Create Sensor Instance
	result = GetDefaultKinectSensor(&sensor);
	if (FAILED(result)) {
		throw std::exception("Exception : GetDefaultKinectSensor()");
	}

	// Open Sensor
	result = sensor->Open();
	if (FAILED(result)) {
		throw std::exception("Exception : IKinectSensor::Open()");
	}

	// Retrieved Depth Frame Source
	result = sensor->get_DepthFrameSource(&depthSource);
	if (FAILED(result)) {
		throw std::exception("Exception : IKinectSensor::get_DepthFrameSource()");
	}

	// Retrieved Depth Frame Size
	IFrameDescription* depthDescription;
	result = depthSource->get_FrameDescription(&depthDescription);
	if (FAILED(result)) {
		throw std::exception("Exception : IDepthFrameSource::get_FrameDescription()");
	}

	result = depthDescription->get_Width(&depthWidth); // 512
	if (FAILED(result)) {
		throw std::exception("Exception : IFrameDescription::get_Width()");
	}

	result = depthDescription->get_Height(&depthHeight); // 424
	if (FAILED(result)) {
		throw std::exception("Exception : IFrameDescription::get_Height()");
	}

	if (depthDescription != NULL) {
		depthDescription->Release();
		depthDescription = NULL;
	}

	// To Reserve Depth Frame Buffer
	depthBuffer.resize(depthWidth * depthHeight);
}

KinectGrabber::~KinectGrabber() throw()
{
	stop();
	thread.join();
	if (sensor) {
		sensor->Close();
	}
	if (sensor != NULL) {
		sensor->Release();
		sensor = NULL;
	}
	if (depthSource != NULL) {
		depthSource->Release();
		depthSource = NULL;
	}
	if (depthReader != NULL) {
		depthReader->Release();
		depthReader = NULL;
	}
}

void KinectGrabber::start() {
	// Open Depth Frame Reader
	result = depthSource->OpenReader(&depthReader);
	if (FAILED(result)) {
		throw std::exception("Exception : IDepthFrameSource::OpenReader()");
	}

	running = true;

	thread = boost::thread(&KinectGrabber::threadFunction, this);
}

bool KinectGrabber::isDataAvailable()
{
	return dataAvailable;
}

cv::Mat KinectGrabber::getMat()
{
	return cv::Mat(depthHeight, depthWidth, CV_8U, &depthBuffer);
}

void KinectGrabber::stop()
{
	boost::unique_lock<boost::mutex> lock(mutex);
	running = false;
	lock.unlock();
}

size_t KinectGrabber::getFrameSize()
{
	return size_t(depthBuffer.size());
}

void KinectGrabber::threadFunction()
{
	while (running)
	{
		boost::unique_lock<boost::mutex> lock(mutex);
		// Acquire Latest Depth Frame
		IDepthFrame* depthFrame = nullptr;
		result = depthReader->AcquireLatestFrame(&depthFrame);
		if (SUCCEEDED(result)) {
			// Retrieved Depth Data
			result = depthFrame->CopyFrameDataToArray(depthBuffer.size(), &depthBuffer[0]);
			if (FAILED(result)) {
				throw std::exception("Exception : IDepthFrame::CopyFrameDataToArray()");
			}
			dataAvailable = true;
		}
		if (depthFrame != NULL)
		{
			depthFrame->Release();
			depthFrame = NULL;
		}
		lock.unlock();
	}
}

bool KinectGrabber::fillFrame(cv::Mat * destData)
{
	if (dataAvailable) {
		for (size_t i = 0; i < depthBuffer.size(); i++) {
			double depth = depthBuffer[i];
			if (maxZ > 0 && depth > maxZ) {
				depth = maxZ;
			}
			if (minZ > 0 && minZ > depth) {
				depth = minZ;
			}
			depth = depth - minZ;
			if (maxZ > minZ) {
				depth = depth / (maxZ - minZ);
			}
			else {
				depth = depth / 4000.;
			}
			//depth = depth * USHRT_MAX;
			//destData->at<UINT16>(i) = depth;

			depth = depth * UCHAR_MAX;
			destData->at<UINT8>(i) = depth;

		}
	}
	return dataAvailable;
	dataAvailable = false;
}