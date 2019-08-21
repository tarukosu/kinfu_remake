#ifndef __AZURE_KINECT_GRABBER_SOURCE__
#define __AZURE_KINECT_GRABBER_SOURCE__

#include <kfusion/kinfu.hpp>
#include <opencv2/core/core.hpp>
#include <string>
#include <fstream>
#include <k4a/k4a.h>
#include "io/bin_grabber.hpp"

namespace kfusion {
	class KF_EXPORTS AzureKinectGrabber : public Grabber {
	public:
		void release();

		AzureKinectGrabber();
		~AzureKinectGrabber();

		bool grab(cv::Mat &depth, cv::Mat &image);
		bool setRegistration(bool value = false);

		Intr getIntr();

	private:
		int returnCode = 1;
		k4a_device_t device = NULL;
		const int32_t TIMEOUT_IN_MS = 1000;
		k4a_capture_t capture = NULL;
		k4a_transformation_t transformation = NULL;

		int capture_count = 0;


		k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
		k4a_image_t color_image = NULL;
		k4a_image_t depth_image = NULL;

		k4a_image_t point_cloud = NULL;
		int point_count = 0;

		k4a_image_t point_cloud_image = NULL;

		uint8_t* color_image_data = NULL;

		int width = 0;
		int height = 0;

		cv::Mat color_mat;



	};
}

#endif
