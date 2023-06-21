// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Associated header
//
#include "k4abodytrackermanager.h"




K4ABodyTrackerManager &K4ABodyTrackerManager::Instance()
{
	static K4ABodyTrackerManager instance;
	return instance;
}

k4a_result_t K4ABodyTrackerManager::StartBodyTracker(k4a::device& device, k4a_device_configuration_t& deviceConfig)
{
	k4a_device_get_calibration(device.handle(), deviceConfig.depth_mode, deviceConfig.color_resolution, &m_calibration);
	k4abt_tracker_configuration_t trackerConfig = K4ABT_TRACKER_CONFIG_DEFAULT;
	k4a_result_t result = k4abt_tracker_create(&m_calibration, trackerConfig, &m_tracker);
	if (result == K4A_RESULT_FAILED) {
		//printf("ERROR! Body tracker initialization failed!");
	}
	return result;
}

k4a_wait_result_t K4ABodyTrackerManager::EnqueueCaptureResult(const k4a::capture & capture)
{
	k4a_wait_result_t result = k4abt_tracker_enqueue_capture(m_tracker, capture.handle(), 0);
	if (result == K4A_WAIT_RESULT_FAILED) {
		//printf("ERROR! Adding capture to tracker queue failed!");
	}
	return result;
}
k4a_wait_result_t K4ABodyTrackerManager::PopBodyFrameResult(k4abt_frame_t& bodyFrame)
{
	k4a_wait_result_t result = k4abt_tracker_pop_result(m_tracker, &m_bodyFrame, 0);
	if (result == K4A_WAIT_RESULT_FAILED) {
		printf("ERROR! Getting body frame result failed!");
	}
	else {
		bodyFrame = m_bodyFrame;
	}
	return result;
}

k4a_calibration_t K4ABodyTrackerManager::GetCameraCalibration() {
	return m_calibration;
}