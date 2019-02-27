/* robogame_kinectfeatures_extractor
 * utils.cpp -- This file is part of the robogame_kinectfeatures_extractor ROS node created for
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

#include <ros/ros.h>    // ROS
#include <math.h>       /* ceil */
#include <string>
#include "utils.h"
#include "common.h"

/* printMatImage -- a function to print a cv::Mat to console. Used
mostly for debugging purposes*/
void printMatImage(cv::Mat frame){
    printf("\n --> Matrix type is CV_32F \n");

    for( size_t i = 0; i < frame.rows; i++ ) {
        for( size_t j = 0; j < frame.cols; j++ ) {
   	    printf( " %.3f  ", frame.at<float>(i,j) );
 	}
	printf("\n");
    }
}

/* writeMatToFile -- a function to save a cv::Mat to file. Mainly
used for frame saving purposes.*/
void writeMatToFile(cv::Mat& frame, const char* filename)
{
    std::ofstream fout(filename);           // Define file.
    if(!fout){
        std::cout << "File Not Opened" << std::endl;  return;
    }
    /*Loop through pixels in the cv::Mat file*/
    for(unsigned int i=0; i<frame.rows; i++){
        for(unsigned int j=0; j<frame.cols; j++){
            fout << frame.at<float>(i,j) << " ";
        }
        fout << std::endl;
    }
    fout.close();                           // close file
}


/* distanceFunction -- used to compute the similarity betweent the pixels.*/
bool distanceFunction(float a, float b, int threshold){
    if(abs(a-b) <= threshold) return true;
	else return false;
}