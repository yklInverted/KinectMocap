#include "cv_utils.h"

void ColorImageToMat(k4a_image_t c_image, cv::Mat& c_mat) {

	int height = k4a_image_get_height_pixels(c_image);
	int width = k4a_image_get_width_pixels(c_image);
	//int stride = k4a_image_get_stride_bytes(c_image);

	c_mat = cv::Mat(height, width, CV_8UC4);

	size_t szInBytes = k4a_image_get_size(c_image);
	uint8_t* buffer = k4a_image_get_buffer(c_image);


	memcpy(c_mat.data, buffer, szInBytes);
	//k4a_image_release(c_image);
}

void  MatToColorImage(cv::Mat& c_mat, k4a_image_t& c_image, k4a_image_format_t format) {

	int width = c_mat.cols;
	int height = c_mat.rows;


	size_t szInBytes = width * height * c_mat.elemSize();
	printf("width: %d, height: %d, size: %d \n", width, height, szInBytes);

	k4a_image_create_from_buffer(format, width, height, 0,
		c_mat.data, szInBytes, NULL, NULL, &c_image);
}

void Visualize2DBody(k4abt_frame_t& body_frame, cv::Mat& c_mat, k4a_calibration_t sensor_calibration) {

	k4a_capture_t original_capture = k4abt_frame_get_capture(body_frame);
	k4a_image_t depth_image = k4a_capture_get_depth_image(original_capture);


	uint32_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
	for (uint32_t i = 0; i < num_bodies; i++) {
		k4abt_body_t body;
		VERIFY(k4abt_frame_get_body_skeleton(body_frame, i, &body.skeleton), "Get skeleton from body frame failed!");
		body.id = k4abt_frame_get_body_id(body_frame, i);

		Body2D body2d_color(sensor_calibration, K4A_CALIBRATION_TYPE_COLOR, int(body.id));
		Body2D body2d_depth(sensor_calibration, K4A_CALIBRATION_TYPE_DEPTH, int(body.id));

		// Assign the correct color based on the body id
		Color color = g_bodyColors[body.id % g_bodyColors.size()];
		color.a = 0.4f;
		Color low_confidence_color = color;
		low_confidence_color.a = 0.1f;

		// Visualize joints
		for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++) {
			if (body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW) {


				k4a_float2_t joint_in_color_2d;
				bool valid = transform_joint_from_depth_3d_to_color_2d(&sensor_calibration, body.skeleton.joints[joint].position, joint_in_color_2d);
				if (valid) {
					body2d_color.skeleton2d.joints2d[joint].x = joint_in_color_2d.v[0];
					body2d_color.skeleton2d.joints2d[joint].y = joint_in_color_2d.v[1];
					body2d_color.skeleton2d.joints2d[joint].confidence_level = body.skeleton.joints[joint].confidence_level;
					body2d_color.skeleton2d.joints2d[joint].confident = body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM;
					body2d_color.skeleton2d.joints2d[joint].color = body2d_color.skeleton2d.joints2d[joint].confident ? cv::Scalar(uint8_t(color.b * 255), uint8_t(color.g * 255), uint8_t(color.r * 255), uint8_t(color.a * 255)) : cv::Scalar(uint8_t(low_confidence_color.b * 255), uint8_t(low_confidence_color.g * 255), uint8_t(low_confidence_color.r * 255), uint8_t(low_confidence_color.a * 255));

					cv::circle(c_mat, cv::Point2f(joint_in_color_2d.v[0], joint_in_color_2d.v[1]), 3, body2d_color.skeleton2d.joints2d[joint].color, 3);
				}

			}
		}

		// Visualize bones
		for (size_t bone_idx = 0; bone_idx < g_boneList.size(); bone_idx++) {
			k4abt_joint_id_t joint1 = g_boneList[bone_idx].first;
			k4abt_joint_id_t joint2 = g_boneList[bone_idx].second;

			if (body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW &&
				body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW) {
				bool confident_bone = (body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM) && (body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM);


				Joint2D joint2d1 = body2d_color.skeleton2d.joints2d[joint1];
				Joint2D joint2d2 = body2d_color.skeleton2d.joints2d[joint2];
				if (!((joint2d1.x == 0 && joint2d1.y == 0) || (joint2d2.x == 0 && joint2d2.y == 0))) cv::line(c_mat, cv::Point2f(joint2d1.x, joint2d1.y), cv::Point2f(joint2d2.x, joint2d2.y), confident_bone ? cv::Scalar(uint8_t(color.b * 255), uint8_t(color.g * 255), uint8_t(color.r * 255), uint8_t(color.a * 255)) : cv::Scalar(uint8_t(low_confidence_color.b * 255), uint8_t(low_confidence_color.g * 255), uint8_t(low_confidence_color.r * 255), uint8_t(low_confidence_color.a * 255)), 3);

			}
		}
	}
	//k4a_capture_release(original_capture);
	k4a_image_release(depth_image);
}


void Visualize3DBody(k4abt_frame_t& body_frame, cv::Mat& d_mat, k4a_calibration_t sensor_calibration) {

	k4a_capture_t original_capture = k4abt_frame_get_capture(body_frame);
	k4a_image_t depth_image = k4a_capture_get_depth_image(original_capture);

	/*
	k4a_image_t body_index_map = k4abt_frame_get_body_index_map(body_frame);
	const uint8_t* body_index_map_buffer = k4a_image_get_buffer(body_index_map);

	
	if (!d_mat.empty()) {
		for (int i = 0; i < d_mat.rows; ++i) {
			for (int j = 0; j < d_mat.cols; ++j) {
				uint8_t body_index = body_index_map_buffer[j + i * d_mat.cols];
				if (body_index != K4ABT_BODY_INDEX_MAP_BACKGROUND) {
					uint32_t body_id = k4abt_frame_get_body_id(body_frame, body_index);
					Color tmp = g_bodyColors[body_id % g_bodyColors.size()];
					d_mat.at<cv::Vec4b>(i, j)[0] = d_mat.at<cv::Vec4b>(i, j)[0] * 0.75 + uchar(tmp.b * 255) * 0.25;
					d_mat.at<cv::Vec4b>(i, j)[1] = d_mat.at<cv::Vec4b>(i, j)[1] * 0.75 + uchar(tmp.g * 255) * 0.25;
					d_mat.at<cv::Vec4b>(i, j)[2] = d_mat.at<cv::Vec4b>(i, j)[2] * 0.75 + uchar(tmp.r * 255) * 0.25;
					// d_mat.at<cv::Vec4b>(i, j)[3] = uchar(tmp.a * 255);
				}
			}
		}
	}
	k4a_image_release(body_index_map);
	*/
	uint32_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
	for (uint32_t i = 0; i < num_bodies; i++) {
		k4abt_body_t body;
		VERIFY(k4abt_frame_get_body_skeleton(body_frame, i, &body.skeleton), "Get skeleton from body frame failed!");
		body.id = k4abt_frame_get_body_id(body_frame, i);

		Body2D body2d_color(sensor_calibration, K4A_CALIBRATION_TYPE_COLOR, int(body.id));
		Body2D body2d_depth(sensor_calibration, K4A_CALIBRATION_TYPE_DEPTH, int(body.id));

		// Assign the correct color based on the body id
		Color color = g_bodyColors[body.id % g_bodyColors.size()];
		color.a = 0.4f;
		Color low_confidence_color = color;
		low_confidence_color.a = 0.1f;

		// Visualize joints
		for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++) {
			if (body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW) {

				k4a_float2_t joint_in_depth_2d;

				// else std::cerr << "Invalid Pixel Locations" << std::endl;
				bool valid = transform_joint_from_depth_3d_to_depth_2d(&sensor_calibration, body.skeleton.joints[joint].position, joint_in_depth_2d);
				if (valid) {
					body2d_depth.skeleton2d.joints2d[joint].x = joint_in_depth_2d.v[0];
					body2d_depth.skeleton2d.joints2d[joint].y = joint_in_depth_2d.v[1];
					body2d_depth.skeleton2d.joints2d[joint].confidence_level = body.skeleton.joints[joint].confidence_level;
					body2d_depth.skeleton2d.joints2d[joint].confident = body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM;
					body2d_depth.skeleton2d.joints2d[joint].color = body2d_depth.skeleton2d.joints2d[joint].confident ? cv::Scalar(uint8_t(color.b * 255), uint8_t(color.g * 255), uint8_t(color.r * 255), uint8_t(color.a * 255)) : cv::Scalar(uint8_t(low_confidence_color.b * 255), uint8_t(low_confidence_color.g * 255), uint8_t(low_confidence_color.r * 255), uint8_t(low_confidence_color.a * 255));

					cv::circle(d_mat, cv::Point2f(joint_in_depth_2d.v[0], joint_in_depth_2d.v[1]), 3, body2d_depth.skeleton2d.joints2d[joint].color, 3);
				}
			}
		}

		// Visualize bones
		for (size_t bone_idx = 0; bone_idx < g_boneList.size(); bone_idx++) {
			k4abt_joint_id_t joint1 = g_boneList[bone_idx].first;
			k4abt_joint_id_t joint2 = g_boneList[bone_idx].second;

			if (body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW &&
				body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW) {
				bool confident_bone = (body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM) && (body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM);


				Joint2D joint2d1 = body2d_depth.skeleton2d.joints2d[joint1];
				Joint2D joint2d2 = body2d_depth.skeleton2d.joints2d[joint2];
				if (!((joint2d1.x == 0 && joint2d1.y == 0) || (joint2d2.x == 0 && joint2d2.y == 0))) cv::line(d_mat, cv::Point2f(joint2d1.x, joint2d1.y), cv::Point2f(joint2d2.x, joint2d2.y), confident_bone ? cv::Scalar(uint8_t(color.b * 255), uint8_t(color.g * 255), uint8_t(color.r * 255), uint8_t(color.a * 255)) : cv::Scalar(uint8_t(low_confidence_color.b * 255), uint8_t(low_confidence_color.g * 255), uint8_t(low_confidence_color.r * 255), uint8_t(low_confidence_color.a * 255)), 3);
			}
		}
	}
	//k4a_capture_release(original_capture);
	k4a_image_release(depth_image);
}