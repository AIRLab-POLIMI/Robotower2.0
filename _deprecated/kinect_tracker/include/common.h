/* commons.h -- This file is part of the kinect_feat_extractor ROS node created for
 * the purpose of extracting relevant motion features from images.
 *
 * Copyright (C) 2016 Ewerton Lopes
 *
 * LICENSE: This is a free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License (LGPL v3) as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version. This code is distributed in the hope
 * that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU Lesser General Public License (LGPL v3): http://www.gnu.org/licenses/.
 */

#ifndef COMMON_H
#define COMMON_H

#include <boost/circular_buffer.hpp>
#include <opencv2/opencv.hpp>

extern cv::Point2f blobCenter;
extern bool isPlayerMissing;
extern cv::Mat segmentedColorFrame;
extern cv::Mat segmentedTarget;

extern const int BUFFER; // the object trail size
extern boost::circular_buffer<cv::Point2f> pts;

#endif
