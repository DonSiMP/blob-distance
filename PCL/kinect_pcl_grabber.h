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

#ifndef KINECT2_GRABBER
#define KINECT2_GRABBER

#define NOMINMAX
#include <Windows.h>
#include <Kinect.h>

#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


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

	class Kinect2Grabber : public pcl::Grabber
	{
	public:
		Kinect2Grabber();
		virtual ~Kinect2Grabber() throw ();
		virtual void start();
		virtual void stop();
		virtual bool isRunning() const;
		virtual std::string getName() const;
		virtual float getFramesPerSecond() const;

		typedef void (signal_Kinect2_PointXYZ)(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>&);
		typedef void (signal_Range_Def)(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>&, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>&);

	protected:
		boost::signals2::signal<signal_Kinect2_PointXYZ>* signal_PointXYZ;
		boost::signals2::signal<signal_Range_Def>* signal_Range;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMin, cloudMax;

		pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPointXYZ(UINT16* depthBuffer);
		pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPointXYZ_perf(UINT16* depthBuffer);
		pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPointXYZ_orig(UINT16* depthBuffer);

		boost::thread thread;
		mutable boost::mutex mutex;

		void threadFunction();

		bool quit;
		bool running;

		HRESULT result;
		IKinectSensor* sensor;
		ICoordinateMapper* mapper;
		IDepthFrameSource* depthSource;
		IDepthFrameReader* depthReader;

		int depthWidth;
		int depthHeight;
		std::vector<UINT16> depthBuffer;

	};

	pcl::Kinect2Grabber::Kinect2Grabber()
		: sensor(nullptr)
		, mapper(nullptr)
		, depthSource(nullptr)
		, depthReader(nullptr)
		, result(S_OK)
		, depthWidth(512)
		, depthHeight(424)
		, depthBuffer()
		, running(false)
		, quit(false)
		, signal_PointXYZ(nullptr)
		, signal_Range(nullptr)
		, cloudMin(new pcl::PointCloud<pcl::PointXYZ>())
		, cloudMax(new pcl::PointCloud<pcl::PointXYZ>())
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

		// Retrieved Coordinate Mapper
		result = sensor->get_CoordinateMapper(&mapper);
		if (FAILED(result)) {
			throw std::exception("Exception : IKinectSensor::get_CoordinateMapper()");
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

		SafeRelease(depthDescription);

		// To Reserve Depth Frame Buffer
		depthBuffer.resize(depthWidth * depthHeight);

		signal_PointXYZ = createSignal<signal_Kinect2_PointXYZ>();
		signal_Range = createSignal<signal_Range_Def>();

		cloudMin->points.resize(1);
		cloudMax->points.resize(1);
	}

	pcl::Kinect2Grabber::~Kinect2Grabber() throw()
	{
		stop();

		disconnect_all_slots<signal_Kinect2_PointXYZ>();

		thread.join();

		// End Processing
		if (sensor) {
			sensor->Close();
		}
		SafeRelease(sensor);
		SafeRelease(mapper);
		SafeRelease(depthSource);
		SafeRelease(depthReader);
	}

	void pcl::Kinect2Grabber::start()
	{
		// Open Depth Frame Reader
		result = depthSource->OpenReader(&depthReader);
		if (FAILED(result)) {
			throw std::exception("Exception : IDepthFrameSource::OpenReader()");
		}

		running = true;

		thread = boost::thread(&Kinect2Grabber::threadFunction, this);
	}

	void pcl::Kinect2Grabber::stop()
	{
		boost::unique_lock<boost::mutex> lock(mutex);

		quit = true;
		running = false;

		lock.unlock();
	}

	bool pcl::Kinect2Grabber::isRunning() const
	{
		boost::unique_lock<boost::mutex> lock(mutex);

		return running;

		lock.unlock();
	}

	std::string pcl::Kinect2Grabber::getName() const
	{
		return std::string("Kinect2Grabber");
	}

	float pcl::Kinect2Grabber::getFramesPerSecond() const
	{
		return 30.0f;
	}

	void pcl::Kinect2Grabber::threadFunction()
	{
		while (!quit) {
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
			}
			SafeRelease(depthFrame);

			lock.unlock();

			if (signal_PointXYZ->num_slots() > 0) {
				signal_PointXYZ->operator()(convertDepthToPointXYZ(&depthBuffer[0]));
			}

		}
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl::Kinect2Grabber::convertDepthToPointXYZ(UINT16* depthBuffer)
	{
		return pcl::Kinect2Grabber::convertDepthToPointXYZ_perf(depthBuffer);
	}


	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl::Kinect2Grabber::convertDepthToPointXYZ_perf(UINT16* depthBuffer)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>());
			cloud_p->width = static_cast<uint32_t>(depthWidth);
			cloud_p->height = static_cast<uint32_t>(depthHeight);
			cloud_p->is_dense = false;

			cloud_p->points.resize(cloud_p->height * cloud_p->width);
		
		if (signal_Range->num_slots() > 0) {
			signal_Range->operator()(cloudMin, cloudMax);
		}
		
		pcl::PointXYZ* pt = &cloud_p->points[0];
		size_t validCnt = 0;
		for (int y = 0; y < depthHeight; y++) {
			for (int x = 0; x < depthWidth; x++) {
				//pcl::PointXYZ point;

				DepthSpacePoint depthSpacePoint = { static_cast<float>(x), static_cast<float>(y) };
				UINT16 depth = depthBuffer[y * depthWidth + x];

				// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
				CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
				mapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint);

				bool validx = (cloudMin->points[0].x == cloudMax->points[0].x
					|| (cloudMin->points[0].x <= cameraSpacePoint.X && cloudMax->points[0].x >= cameraSpacePoint.X));
				bool validy = (cloudMin->points[0].y == cloudMax->points[0].y
					|| (cloudMin->points[0].y <= cameraSpacePoint.Y && cloudMax->points[0].y >= cameraSpacePoint.Y));
				bool validz = (cloudMin->points[0].z == cloudMax->points[0].z
					|| (cloudMin->points[0].z <= cameraSpacePoint.Z && cloudMax->points[0].z >= cameraSpacePoint.Z));

				bool validPoint = validx && validy && validz;
				if (validPoint) {
					pt->x = cameraSpacePoint.X;
					pt->y = cameraSpacePoint.Y;
					pt->z = cameraSpacePoint.Z;
					pt++;
					validCnt++;
				}

			}
		}
		cloud_p->points.resize(validCnt);
		return cloud_p;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl::Kinect2Grabber::convertDepthToPointXYZ_orig(UINT16* depthBuffer)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

		cloud->width = static_cast<uint32_t>(depthWidth);
		cloud->height = static_cast<uint32_t>(depthHeight);
		cloud->is_dense = false;

		cloud->points.resize(cloud->height * cloud->width);

		pcl::PointXYZ* pt = &cloud->points[0];
		for (int y = 0; y < depthHeight; y++) {
			for (int x = 0; x < depthWidth; x++, pt++) {
				pcl::PointXYZ point;

				DepthSpacePoint depthSpacePoint = { static_cast<float>(x), static_cast<float>(y) };
				UINT16 depth = depthBuffer[y * depthWidth + x];

				// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
				CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
				mapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint);
				point.x = cameraSpacePoint.X;
				point.y = cameraSpacePoint.Y;
				point.z = cameraSpacePoint.Z;

				*pt = point;
			}
		}

		return cloud;
	}

}

#endif KINECT2_GRABBER

