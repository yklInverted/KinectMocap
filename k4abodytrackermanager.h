// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef K4ABODYTRACKERMANAGER_H
#define K4ABODYTRACKERMANAGER_H

// System headers
//

// Library headers
//
#include <k4a/k4a.hpp>
#include <k4abt.hpp>
// Project headers
//


// Singleton that holds info on the last error detected
//
class K4ABodyTrackerManager
{
public:
	static K4ABodyTrackerManager& Instance();
	k4a_result_t StartBodyTracker(k4a::device& device, k4a_device_configuration_t& deviceConfig);
	k4a_wait_result_t EnqueueCaptureResult(const k4a::capture & capture);
	k4a_wait_result_t PopBodyFrameResult(k4abt_frame_t& bodyFrame);
	k4a_calibration_t GetCameraCalibration();

private:
	K4ABodyTrackerManager() = default;
	
	k4a_calibration_t m_calibration;
	k4abt_tracker_t m_tracker;
	k4abt_frame_t m_bodyFrame = NULL;
};

// namespace k4aviewer

#endif
