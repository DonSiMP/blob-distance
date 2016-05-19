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
#include "../duo/DUO.h"
#include <Dense3D.h>

#include <opencv2/core/mat.hpp>
#include <boost/thread.hpp>
#include <limits.h>

#define WIDTH	752//320
#define HEIGHT	480//240
#define FPS		30

typedef unsigned short      UINT16;
typedef unsigned char       UINT8;

template<class Interface>
inline void SafeRelease(Interface *& IRelease)
{
	if (IRelease != NULL) {
		IRelease->Release();
		IRelease = NULL;
	}
}

class DuoGrabber
{
public:
	DuoGrabber();
	~DuoGrabber();
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
	DUOInstance _duo = NULL;
	PDUOFrame _pFrameData = NULL;
	HANDLE _evFrame = CreateEvent(NULL, FALSE, FALSE, NULL);
	Dense3DInstance dense3d;
	// Create Mat for disparity and depth map
	cv::Mat1f disparity = cv::Mat(cv::Size(WIDTH, HEIGHT), CV_32FC1);
	cv::Mat3f depth3d = cv::Mat(cv::Size(WIDTH, HEIGHT), CV_32FC3);

	cv::Mat left;
	cv::Mat right;
	cv::Mat l, r, disp, disp8;
	std::vector<UINT16> depthBuffer;

	boost::thread thread;
	mutable boost::mutex mutex;
	bool running;
	bool dataAvailable;

	int numDisparities = 16;
	int blockSize = 21;
};

DuoGrabber::DuoGrabber()
	: _duo(nullptr)
	, _pFrameData(nullptr)
	, _evFrame(nullptr)
	, dense3d(nullptr)
	, depthWidth(WIDTH)
	, depthHeight(HEIGHT)
	, depthBuffer()
	, running(false)
	, dataAvailable(false)
	, minZ(0.)
	, maxZ(0.)
{
	// Open DUO camera and start capturing
	if (!OpenDUOCamera(WIDTH, HEIGHT, FPS))
	{
		std::exception("Could not open DUO camera\n");

	}
	// Open Dense3D
	if (!Dense3DOpen(&dense3d))
	{
		std::exception("Could not open Dense3D library\n");
		// Close DUO camera
		CloseDUOCamera();
	}
	// Set the Dense3D license (visit https://duo3d.com/account)
	if (!SetDense3DLicense(dense3d, "3MA6R-9RY8M-PNL2D-OONBL-XH18V")) // <-- Put your Dense3D license
	{
		std::exception("Invalid or missing Dense3D license. To get your license visit https://duo3d.com/account\n");
		// Close DUO camera
		CloseDUOCamera();
		// Close Dense3D library
		Dense3DClose(dense3d);
	}
	// Set the image size
	if (!SetDense3DImageSize(dense3d, WIDTH, HEIGHT))
	{
		std::exception("Invalid image size\n");
		// Close DUO camera
		CloseDUOCamera();
		// Close Dense3D library
		Dense3DClose(dense3d);
	}
	// Get DUO calibration intrinsics and extrinsics
	INTRINSICS intr;
	EXTRINSICS extr;
	if (!GetCameraParameters(&intr, &extr))
	{
		std::exception("Could not get DUO camera calibration data\n");
		// Close DUO camera
		CloseDUOCamera();
		// Close Dense3D library
		Dense3DClose(dense3d);
	}
	// Set Dense3D parameters
	SetDense3DMode(dense3d, 0);
	SetDense3DCalibration(dense3d, &intr, &extr);
	SetDense3DNumDisparities(dense3d, 2);
	SetDense3DSADWindowSize(dense3d, 6);
	SetDense3DPreFilterCap(dense3d, 28);
	SetDense3DUniquenessRatio(dense3d, 27);
	SetDense3DSpeckleWindowSize(dense3d, 52);
	SetDense3DSpeckleRange(dense3d, 14);
	// Set exposure, LED brightness and camera orientation
	SetExposure(85);
	//SetLed(28);
	SetLed(255);
	SetVFlip(true);
	// Enable retrieval of undistorted (rectified) frames
	SetUndistort(true);
	// Create Mat for left & right frames
	left.create(cv::Size(WIDTH, HEIGHT), CV_8UC1);
	right.create(cv::Size(WIDTH, HEIGHT), CV_8UC1);

	// To Reserve Depth Frame Buffer
	depthBuffer.resize(depthWidth * depthHeight);
}

DuoGrabber::~DuoGrabber() throw()
{
	stop();
	thread.join();

	// End Processing
	// Close DUO camera
	CloseDUOCamera();
	// Close Dense3D library
	Dense3DClose(dense3d);
}

void DuoGrabber::start() {
	running = true;

	thread = boost::thread(&DuoGrabber::threadFunction, this);
	cv::namedWindow("controls", 1);
	cv::createTrackbar("numDisparities", "controls", &numDisparities, 100);
	cv::createTrackbar("blockSize", "controls", &blockSize, 101);
}

bool DuoGrabber::isDataAvailable()
{
	return dataAvailable;
}

std::vector<UINT16> * DuoGrabber::getDepthData()
{
	return &depthBuffer;
}

void DuoGrabber::stop()
{
	boost::unique_lock<boost::mutex> lock(mutex);
	running = false;
	lock.unlock();
}

size_t DuoGrabber::getFrameSize()
{
	return size_t(depthBuffer.size());
}

void DuoGrabber::threadFunction()
{
	while (running)
	{
		boost::unique_lock<boost::mutex> lock(mutex);

		// Capture DUO frame
		PDUOFrame pFrameData = GetDUOFrame();
		if (pFrameData == NULL) continue;

		// Set the image data
		left.data = (uchar*)pFrameData->leftData;
		left.copyTo(l);
		right.data = (uchar*)pFrameData->rightData;
		right.copyTo(r);
		lock.unlock();
 		cv::Ptr<cv::StereoMatcher> stereo = cv::StereoSGBM::create(0, numDisparities, blockSize);
			//cv::StereoBM::create(numDisparities, blockSize);
		stereo->compute(left, right, disp);
		disp.convertTo(disp8, CV_8U);
		dataAvailable = true;
		


		if (false && Dense3DGetDepth(dense3d, pFrameData->leftData, pFrameData->rightData, (float*)disparity.data, (PDense3DDepth)depth3d.data)) {
			UINT16 * ptr = &depthBuffer[0];
			PDense3DDepth depthData = (PDense3DDepth)depth3d.data;
			for (size_t i = 0; i < depthBuffer.size(); i++, depthData++, ptr++)
			{
				float length = sqrt(pow(depthData->x, 2) + pow(depthData->y, 2) + pow(depthData->z, 2));
				*ptr = (UINT16)(length);
			}
			dataAvailable = true;
		}
	}
}

double DuoGrabber::getRelativeValue(double depth) {
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

bool DuoGrabber::fillFrame(cv::Mat * destData)
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
		cv::imshow("Left", l);
		cv::imshow("Right", r);
		cv::imshow("Disparity", disp8);
		return true;
	}
	return false;
}