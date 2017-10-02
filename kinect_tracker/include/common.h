/* commons.h -- This file is part of the kinect_tracker ROS node created for
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

#define MIN_DISTANCE 0.3
#define MAX_DISTANCE 4.2
#define TRAIL_BUFFER_SIZE 32


extern int hMin;
extern int sMin;
extern int vMin;
extern int hMax;
extern int sMax;
extern int vMax;

extern cv::Point2f blobCenter;
extern bool isPlayerMissing;
extern cv::Mat segmentedColorFrame;
extern cv::Mat segmentedTarget;

extern const int BUFFER; // the object trail size
extern boost::circular_buffer<cv::Point2f> pts;

#endif
