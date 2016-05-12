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

#define LEAP_GRABBER

#define NOMINMAX
#include <Windows.h>
#include "Leap.h"

#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#define WIDTH 640
#define HEIGHT 240
#define FPS 30

static const int minLight = 60;
bool allSet[640] = { false };

typedef unsigned char uchar;

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

	class LEAPGrabber : public pcl::Grabber
	{
	public:
		LEAPGrabber();
		virtual ~LEAPGrabber() throw ();
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
		
		int findIdx(uchar searchVal, bool* idxs, const uchar* image_buffer, const int start, const int end, const int treshold);
		void threadFunction();

		bool quit;
		bool running;

		Leap::Controller controller;

	};

	int pcl::LEAPGrabber::findIdx(uchar searchVal, bool* idxs, const uchar* image_buffer, const int start, const int end, const int treshold) {
		for (int col = start, idx = 0; col < end; col++, idx++)
		{
			if (idxs[idx])continue;
			uchar val = image_buffer[col];
			if (val < minLight) continue;//to dark
			if (val == searchVal || (val - treshold < searchVal && val + treshold>searchVal)) {
				idxs[idx] = true;
				return col - start;
			}
		}
		return -1;
	}

	pcl::LEAPGrabber::LEAPGrabber()
		: running(false)
		, quit(false)
		, signal_PointXYZ(nullptr)
		, signal_Range(nullptr)
		, cloudMin(new pcl::PointCloud<pcl::PointXYZ>())
		, cloudMax(new pcl::PointCloud<pcl::PointXYZ>())
	{
		controller.setPolicy(Leap::Controller::POLICY_IMAGES);

		signal_PointXYZ = createSignal<signal_PointXYZ_Def>();
		signal_Range = createSignal<signal_Range_Def>();

		cloudMin->points.resize(1);
		cloudMax->points.resize(1);
	}

	pcl::LEAPGrabber::~LEAPGrabber() throw()
	{
		stop();

		disconnect_all_slots<signal_PointXYZ_Def>();
		disconnect_all_slots<signal_Range_Def>();

		thread.join();
	}

	void pcl::LEAPGrabber::start()
	{

		running = true;

		thread = boost::thread(&LEAPGrabber::threadFunction, this);
	}

	void pcl::LEAPGrabber::stop()
	{
		boost::unique_lock<boost::mutex> lock(mutex);

		quit = true;
		running = false;

		lock.unlock();
	}

	bool pcl::LEAPGrabber::isRunning() const
	{
		boost::unique_lock<boost::mutex> lock(mutex);

		return running;

		lock.unlock();
	}

	std::string pcl::LEAPGrabber::getName() const
	{
		return std::string("LEAPGrabber");
	}

	float pcl::LEAPGrabber::getFramesPerSecond() const
	{
		return FPS;
	}

	void pcl::LEAPGrabber::threadFunction()
	{
		while (!quit) {
			boost::unique_lock<boost::mutex> lock(mutex);
			Leap::Frame frame = controller.frame();
			Leap::ImageList images = frame.images();
			if (!images[0].isValid() || !images[1].isValid())continue;
			const uchar* image_bufferL = images[0].data(); // brightness values
			const uchar* image_bufferR = images[1].data(); // brightness values
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
			for (int row = 0; row < HEIGHT; row++)
			{
				std::fill(allSet, allSet + 640, false);//remember which pixel already has its pair
				const int start = row * images[0].width() * images[0].bytesPerPixel();

				for (int col = start; col < start + images[0].width(); col++)
				{
					uchar valL = image_bufferL[col];
					if (valL < minLight) continue;//to dark
					int colR = findIdx(valL, allSet, image_bufferR, start, start + images[0].width(), 20);
					if (colR > 0) {
						Leap::Vector slopes_left = images[0].rectify(Leap::Vector(col - start, row, 0));
						Leap::Vector slopes_right = images[1].rectify(Leap::Vector(colR, row, 0));
						float z = 40 / (slopes_right.x - slopes_left.x);
						float y = z * slopes_right.y;
						float x = z * slopes_right.x - 20;
						//https://developer.leapmotion.com/documentation/csharp/devguide/Leap_Coordinate_Mapping.html
						//Values in millimeter
						//converting to m
						//x = x / 1000.;
						//y = y / 1000.;
						//z = z / 1000.;
						bool validx = (cloudMin->points[0].x == cloudMax->points[0].x
							|| (cloudMin->points[0].x <= x && cloudMax->points[0].x >= x));
						bool validy = (cloudMin->points[0].y == cloudMax->points[0].y
							|| (cloudMin->points[0].y <= y && cloudMax->points[0].y >= y));
						bool validz = (cloudMin->points[0].z == cloudMax->points[0].z
							|| (cloudMin->points[0].z <= z && cloudMax->points[0].z >= z));

						bool validPoint = validx && validy && validz;
						if (validPoint) {
							pt->x = x;
							pt->y = y;
							pt->z = z;
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

