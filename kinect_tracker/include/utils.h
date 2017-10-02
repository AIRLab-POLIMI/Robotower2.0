/*kinect_feat_extractor
 * utils.h -- This file is part of the kinect_feat_extractor ROS node created for
 * the purpose of extracting relevant motion features from humans present on images.
 * It defines a blob detection algorithm used for performing segmentation in the image
 * and extract the contraction index (CI) feature. CI is a measure defined in the domain
 * [0 1] indicated by how much the player is streched (arms and legs open wide).
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

#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <fstream>
#include <queue>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

/* trackUser -- Function used to track color blobs on a RGB image. */
void trackUser(cv::Mat& src);

/* resize -- A helper function to resize image. Here, the width is
default to 512 as it is the width of a kinect frame. That is the only
reason for doing it.*/
extern void resize(cv::Mat& image, cv::Mat& dst, int width, int height);

/* writeMatToFile -- a function to save a cv::Mat to file. Mainly
used for frame saving purposes.*/
void writeMatToFile(cv::Mat& m, const char* filename);

/* printMatImage -- a function to print a cv::Mat to console. It
Basically serve as for debugging purposes*/
void printMatImage(cv::Mat _m);

/* distanceFunction -- used to compute the similarity betweent the pixels.*/
bool distanceFunction(float a, float b, int threshold);

/* segmentDepth -- a function that implements a "Region Growing algorithm", which
 is defined here in a "Breadth-first search" manner.
	sX --> Seed Pixel x value (columns == width)
	sY --> Seed Pixel y value (rows == height)
	threshold --> the value to be used in the call to "distanceFunction" method. If distance
    is less than threshold then recursion proceeds, else stops.
*/
void segmentDepth(cv::Mat& input, cv::Mat& dst, int sX, int sY, float& ci, int threshold);

#endif
