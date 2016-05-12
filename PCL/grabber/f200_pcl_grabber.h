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

#define F200_GRABBER

#define NOMINMAX
#include <Windows.h>
#include "pxcsensemanager.h"
#include "pxcmetadata.h"
#include "pxcprojection.h"

#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define WIDTH	640
#define HEIGHT	480
#define FPS		60

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

	class F200Grabber : public pcl::Grabber
	{
	public:
		F200Grabber();
		virtual ~F200Grabber() throw ();
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

		PXCSenseManager* pp = NULL;
		PXCCaptureManager* cm;
		PXCProjection* proj;
		std::vector<PXCPoint3DF32> vertices;

		HRESULT result;

		int depthWidth;
		int depthHeight;
	};

	pcl::F200Grabber::F200Grabber()
		: pp(nullptr)
		, proj(nullptr)
		, result(S_OK)
		, depthWidth(WIDTH)
		, depthHeight(HEIGHT)
		, running(false)
		, quit(false)
		, signal_PointXYZ(nullptr)
		, signal_Range(nullptr)
		, cloudMin(new pcl::PointCloud<pcl::PointXYZ>())
		, cloudMax(new pcl::PointCloud<pcl::PointXYZ>())
	{
		// Create Sensor Instance
		pp = PXCSenseManager::CreateInstance();
		if (!pp) {
			throw std::exception("Exception : CreateInstance()");
		}

		vertices.resize(WIDTH * HEIGHT);

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
		proj = device->CreateProjection();

		vertices.resize(WIDTH * HEIGHT);

		signal_PointXYZ = createSignal<signal_PointXYZ_Def>();
		signal_Range = createSignal<signal_Range_Def>();

		cloudMin->points.resize(1);
		cloudMax->points.resize(1);
	}

	pcl::F200Grabber::~F200Grabber() throw()
	{
		stop();

		disconnect_all_slots<signal_PointXYZ_Def>();
		disconnect_all_slots<signal_Range_Def>();

		thread.join();

		// End Processing
		if (proj) {
			proj->Release();
		}
		if (pp) {
			pp->Close();
			pp->Release();
		}
	}

	void pcl::F200Grabber::start()
	{

		running = true;

		thread = boost::thread(&F200Grabber::threadFunction, this);
	}

	void pcl::F200Grabber::stop()
	{
		boost::unique_lock<boost::mutex> lock(mutex);

		quit = true;
		running = false;

		lock.unlock();
	}

	bool pcl::F200Grabber::isRunning() const
	{
		boost::unique_lock<boost::mutex> lock(mutex);

		return running;

		lock.unlock();
	}

	std::string pcl::F200Grabber::getName() const
	{
		return std::string("F200Grabber");
	}

	float pcl::F200Grabber::getFramesPerSecond() const
	{
		return FPS;
	}

	void pcl::F200Grabber::threadFunction()
	{
		while (!quit) {
			boost::unique_lock<boost::mutex> lock(mutex);
			pxcStatus sts = pp->AcquireFrame(true);

			if (sts < PXC_STATUS_NO_ERROR) {
				throw std::exception("Exception : pp::AcquireFrame()");
			}
			const PXCCapture::Sample *sample = pp->QuerySample();
			if (sample) {
				sts = proj->QueryVertices(sample->depth, &vertices[0]);

				if (sts < PXC_STATUS_NO_ERROR) {
					//no Vertices available
					continue;
				}
			}
			pp->ReleaseFrame();
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
			for each (PXCPoint3DF32 var in vertices)
			{
				//values are in mm (https://software.intel.com/sites/landingpage/realsense/camera-sdk/v1.1/documentation/html/queryvertices_pxcprojection.html)
				//converting to m
				var.x = var.x / 1000.;
				var.y = var.y / 1000.;
				var.z = var.z / 1000.;
				bool validx = (cloudMin->points[0].x == cloudMax->points[0].x
					|| (cloudMin->points[0].x <= var.x && cloudMax->points[0].x >= var.x));
				bool validy = (cloudMin->points[0].y == cloudMax->points[0].y
					|| (cloudMin->points[0].y <= var.y && cloudMax->points[0].y >= var.y));
				bool validz = (cloudMin->points[0].z == cloudMax->points[0].z
					|| (cloudMin->points[0].z <= var.z && cloudMax->points[0].z >= var.z));

				bool validPoint = validx && validy && validz;
				if (validPoint) {
					pt->x = var.x;
					pt->y = var.y;
					pt->z = var.z;
					pt++;
					validCnt++;
				}
			}

			cloud_p->points.resize(validCnt);
			if (signal_PointXYZ->num_slots() > 0) {
				signal_PointXYZ->operator()(cloud_p);
			}

		}
	}

}

