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

#define DUO_GRABBER

#define NOMINMAX
#include <Windows.h>
#include "../DUO/DUO.h"
#include <Dense3D.h>
// Include DUO API header file

#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define WIDTH	752//320
#define HEIGHT	480//240
#define FPS		30

namespace pcl
{
	struct pcl::PointXYZ;
	template <typename T> class pcl::PointCloud;

	template<class Interface>
	inline void SafeRelease(Interface *& IRelease)
	{
		if (IRelease != NULL) {
			IRelease->Release();
			IRelease = NULL;
		}
	}

	class DUOGrabber : public pcl::Grabber
	{
	public:
		DUOGrabber();
		virtual ~DUOGrabber() throw ();
		virtual void start();
		virtual void stop();
		virtual bool isRunning() const;
		virtual std::string getName() const;
		virtual float getFramesPerSecond() const;

		typedef void (signal_PointXYZ_Def)(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>&);
		typedef void (signal_Range_Def)(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>&, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>&);

	protected:
		boost::signals2::signal<signal_PointXYZ_Def>* signal_PointXYZ;
		boost::signals2::signal<signal_Range_Def>* signal_Range;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMin, cloudMax;

		boost::thread thread;
		mutable boost::mutex mutex;

		void threadFunction();

		bool quit;
		bool running;
		
        DUOInstance _duo = NULL;
        PDUOFrame _pFrameData = NULL;
		HANDLE _evFrame = CreateEvent(NULL, FALSE, FALSE, NULL);
		Dense3DInstance dense3d;
		// Create Mat for disparity and depth map
		cv::Mat1f disparity = cv::Mat(cv::Size(WIDTH, HEIGHT), CV_32FC1);
		cv::Mat3f depth3d = cv::Mat(cv::Size(WIDTH, HEIGHT), CV_32FC3);

		cv::Mat left;
		cv::Mat right;

		int depthWidth;
		int depthHeight;
	};

	pcl::DUOGrabber::DUOGrabber()
		: _duo(nullptr)
		, _pFrameData(nullptr)
		, _evFrame(nullptr)
		, dense3d(nullptr)
		, depthWidth(WIDTH)
		, depthHeight(HEIGHT)
		, running(false)
		, quit(false)
		, signal_PointXYZ(nullptr)
		, signal_Range(nullptr)
		, cloudMin(new pcl::PointCloud<pcl::PointXYZ>())
		, cloudMax(new pcl::PointCloud<pcl::PointXYZ>())
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
		SetLed(28);
		SetVFlip(true);
		// Enable retrieval of undistorted (rectified) frames
		SetUndistort(true);
		// Create Mat for left & right frames
		left.create(cv::Size(WIDTH, HEIGHT), CV_8UC1);
		right.create(cv::Size(WIDTH, HEIGHT), CV_8UC1);

		signal_PointXYZ = createSignal<signal_PointXYZ_Def>();
		signal_Range = createSignal<signal_Range_Def>();

		cloudMin->points.resize(1);
		cloudMax->points.resize(1);
	}

	pcl::DUOGrabber::~DUOGrabber() throw()
	{
		stop();

		disconnect_all_slots<signal_PointXYZ_Def>();
		disconnect_all_slots<signal_Range_Def>();

		thread.join();

		// End Processing
		// Close DUO camera
		CloseDUOCamera();
		// Close Dense3D library
		Dense3DClose(dense3d);
	}

	void pcl::DUOGrabber::start()
	{
		running = true;

		thread = boost::thread(&DUOGrabber::threadFunction, this);
	}

	void pcl::DUOGrabber::stop()
	{
		boost::unique_lock<boost::mutex> lock(mutex);

		quit = true;
		running = false;

		lock.unlock();
	}

	bool pcl::DUOGrabber::isRunning() const
	{
		boost::unique_lock<boost::mutex> lock(mutex);

		return running;

		lock.unlock();
	}

	std::string pcl::DUOGrabber::getName() const
	{
		return std::string("DUOGrabber");
	}

	float pcl::DUOGrabber::getFramesPerSecond() const
	{
		return FPS;
	}

	void pcl::DUOGrabber::threadFunction()
	{
		while (!quit) {
			boost::unique_lock<boost::mutex> lock(mutex);

			// Capture DUO frame
			PDUOFrame pFrameData = GetDUOFrame();
			if (pFrameData == NULL) continue;

			// Set the image data
			left.data = (uchar*)pFrameData->leftData;
			right.data = (uchar*)pFrameData->rightData;
			lock.unlock();
			if (signal_Range->num_slots() > 0) {
				signal_Range->operator()(cloudMin, cloudMax);
			}
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>());
			cloud_p->width = static_cast<uint32_t>(WIDTH);
			cloud_p->height = static_cast<uint32_t>(HEIGHT);
			cloud_p->is_dense = false;
			cloud_p->points.resize(cloud_p->height * cloud_p->width);
			pcl::PointXYZ* pt = &cloud_p->points[0];
			size_t validCnt = 0;
			if (Dense3DGetDepth(dense3d, pFrameData->leftData, pFrameData->rightData, (float*)disparity.data, (PDense3DDepth)depth3d.data)) {
				for (int x = 0; x < WIDTH; x++) {
					for (int y = 0; y < HEIGHT; y++) {
						float pX = depth3d.ptr<float>(y, x)[0];
						float pY = depth3d.ptr<float>(y, x)[1];
						float pZ = depth3d.ptr<float>(y, x)[2];
						pX = pX / 1000.;
						pY = pY / 1000.;
						pZ = pZ / 1000.;
						bool validx = (cloudMin->points[0].x == cloudMax->points[0].x
							|| (cloudMin->points[0].x <= pX && cloudMax->points[0].x >= pX));
						bool validy = (cloudMin->points[0].y == cloudMax->points[0].y
							|| (cloudMin->points[0].y <= pY && cloudMax->points[0].y >= pY));
						bool validz = (cloudMin->points[0].z == cloudMax->points[0].z
							|| (cloudMin->points[0].z <= pZ && cloudMax->points[0].z >= pZ));

						bool validPoint = validx && validy && validz;
						if (validPoint) {
							pt->x = pX;
							pt->y = pY;
							pt->z = pZ;
							pt++;
							validCnt++;
						}
					}
				}
			}

			cloud_p->points.resize(validCnt);
			if (signal_PointXYZ->num_slots() > 0) {
				signal_PointXYZ->operator()(cloud_p);
			}

		}
	}

}

