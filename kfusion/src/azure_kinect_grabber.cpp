#include <iostream>
#include <kfusion/kinfu.hpp>
#include <fstream>

#include <opencv2/highgui/highgui.hpp>

#include "io/azure_kinect_grabber.hpp"

namespace kfusion {
	AzureKinectGrabber::AzureKinectGrabber() {
		int device_count = k4a_device_get_installed_count();

		if (device_count == 0)
		{
			printf("No K4A devices found\n");
			throw "Azure kinect devices not found";
		}

		if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
		{
			printf("Failed to open device\n");
			release();
			throw "Failed to open device";
		}

		// ÉJÉÅÉâÇÃèâä˙ê›íË
		config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
		config.color_resolution = K4A_COLOR_RESOLUTION_720P;
		//config.color_resolution = K4A_COLOR_RESOLUTION_3072P;
		config.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
		config.camera_fps = K4A_FRAMES_PER_SECOND_15;
		config.synchronized_images_only = true; // ensures that depth and color images are both available in the capture

		k4a_calibration_t calibration;
		if (K4A_RESULT_SUCCEEDED !=
			k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
		{
			printf("Failed to get calibration\n");
			release();
			throw "Failed to get calibration";
		}

		transformation = k4a_transformation_create(&calibration);

		k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
			calibration.depth_camera_calibration.resolution_width,
			calibration.depth_camera_calibration.resolution_height,
			calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float3_t),
			&point_cloud);

		if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
		{
			printf("Failed to start cameras\n");
			release();
			throw "Failed to start cameras";
		}
	}

	bool AzureKinectGrabber::setRegistration(bool value) {
		return true;
	}

	/*
	#include <math.h>
	#include <string>
	#include <iostream>
	#include <fstream>
	#include <sstream>

	#include <iostream>
	#include <direct.h>

	// opencv
	#include <opencv2/opencv.hpp>

	// pcl
	#ifdef USE_PCL
	#include <pcl/io/pcd_io.h>
	#include <pcl/point_types.h>
	#endif

	// save image
	#include <wrl.h>
	#include <Shlobj.h>
	#include <wincodec.h>
	#include <stdio.h>
	#include <tchar.h>
	#include <locale.h>
	#include <windows.h>
	#include <vector>
	*/

	bool point_cloud_image_depth_to_color(k4a_transformation_t transformation_handle,
		const k4a_image_t depth_image,
		const k4a_image_t color_image,
		k4a_image_t& point_cloud_image)
	{
		// transform color image into depth camera geometry
		int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
		int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
		k4a_image_t transformed_depth_image = NULL;
		if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
			color_image_width_pixels,
			color_image_height_pixels,
			color_image_width_pixels * (int)sizeof(uint16_t),
			&transformed_depth_image))
		{
			printf("Failed to create transformed depth image\n");
			return false;
		}

		// k4a_image_t point_cloud_image = NULL;
		if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
			color_image_width_pixels,
			color_image_height_pixels,
			color_image_width_pixels * 3 * (int)sizeof(int16_t),
			&point_cloud_image))
		{
			k4a_image_release(point_cloud_image);
			printf("Failed to create point cloud image\n");
			return false;
		}

		if (K4A_RESULT_SUCCEEDED !=
			k4a_transformation_depth_image_to_color_camera(transformation_handle, depth_image, transformed_depth_image))
		{
			k4a_image_release(point_cloud_image);
			printf("Failed to compute transformed depth image\n");
			return false;
		}

		if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
			transformed_depth_image,
			K4A_CALIBRATION_TYPE_COLOR,
			point_cloud_image))
		{
			k4a_image_release(point_cloud_image);
			printf("Failed to compute point cloud\n");
			return false;
		}

		k4a_image_release(transformed_depth_image);
		return true;
	}

	/*
	int main(int argc, char **argv)
	{

		// Get a capture
		while (true) {
			switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
			{
			case K4A_WAIT_RESULT_SUCCEEDED:
				break;
			case K4A_WAIT_RESULT_TIMEOUT:
				printf("Timed out waiting for a capture\n");
				goto Exit;
			case K4A_WAIT_RESULT_FAILED:
				printf("Failed to read a capture\n");
				goto Exit;
			}

			capture_count++;

			// Get a depth image
			depth_image = k4a_capture_get_depth_image(capture);
			if (depth_image == 0)
			{
				printf("Failed to get depth image from capture\n");
				goto Exit;
			}

			// Get a color image
			color_image = k4a_capture_get_color_image(capture);
			if (color_image == 0)
			{
				printf("Failed to get color image from capture\n");
				goto Exit;
			}

			if (point_cloud_image_depth_to_color(transformation, depth_image, color_image, point_cloud_image) == false)
			{
				goto Exit;
			}


			if (capture_count > 10) {
				break;
			}
		}

		color_image_data = k4a_image_get_buffer(color_image);

		width = k4a_image_get_width_pixels(color_image);
		height = k4a_image_get_height_pixels(color_image);

		color_mat = cv::Mat(height, width, CV_8UC4, color_image_data);
		cv::imshow("color", color_mat);

		cv::waitKey(33);

	}
	*/
	void AzureKinectGrabber::release() {
		if (point_cloud_image != NULL)
		{
			k4a_image_release(point_cloud_image);
		}
		if (depth_image != NULL)
		{
			k4a_image_release(depth_image);
		}
		if (color_image != NULL)
		{
			k4a_image_release(color_image);
		}
		if (capture != NULL)
		{
			k4a_capture_release(capture);
		}
		if (transformation != NULL)
		{
			k4a_transformation_destroy(transformation);
		}

		if (device != NULL)
		{
			k4a_device_close(device);
		}
	}

	bool AzureKinectGrabber::grab(cv::Mat &depth, cv::Mat &image) {
		switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
		{
		case K4A_WAIT_RESULT_SUCCEEDED:
			break;
		case K4A_WAIT_RESULT_TIMEOUT:
			printf("Timed out waiting for a capture\n");
			return false;
		case K4A_WAIT_RESULT_FAILED:
			printf("Failed to read a capture\n");
			return false;
		}

		// Get a depth image
		depth_image = k4a_capture_get_depth_image(capture);
		if (depth_image == 0)
		{
			printf("Failed to get depth image from capture\n");
			return false;
		}

		// Get a color image
		color_image = k4a_capture_get_color_image(capture);
		if (color_image == 0)
		{
			printf("Failed to get color image from capture\n");
			return false;
		}

		if (point_cloud_image_depth_to_color(transformation, depth_image, color_image, point_cloud_image) == false)
		{
			return false;
		}

		color_image_data = k4a_image_get_buffer(color_image);

		width = k4a_image_get_width_pixels(color_image);
		height = k4a_image_get_height_pixels(color_image);

		color_mat = cv::Mat(height, width, CV_8UC4, color_image_data);
		cv::imshow("color", color_mat);
		cv::waitKey(33);
	}
}
