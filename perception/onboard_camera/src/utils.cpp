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

/* Implements a "Region Growing algorithm", which is defined here in a "Breadth-first search" manner.
	sX --> Seed Pixel x value (columns == width)
	sY --> Seed Pixel y value (rows == height)
	threshold --> the value to be used in the call to "distanceFunction" method. If distance
    is less than threshold then recursion proceeds, else stops.*/
float segmentDepth(cv::Mat& inputFrame, cv::Mat& resultingFrame, int sX, int sY, float& threshold){
	
	long int nPixels = 0;                           			// segmented pixels counter variable.
	std::vector< std::vector<int> > reached;	       			// This is the binary mask for the segmentation.

	for (int i = 0; i < inputFrame.rows; i++){					//They are set to 0 at first. Since no pixel is assigned to the segmentation yet.
		reached.push_back(std::vector<int>(inputFrame.cols));
	}

	// Define the queue. NOTE: it is a BFS based algorithm.
	std::queue< std::pair<int,int> > seg_queue;

	// verify the depth value of the seed position.
	float &in_pxl_pos = inputFrame.at<float>(sY,sX);

    if(in_pxl_pos == 0){
        ROS_WARN_STREAM("THE SEED DEPTH VALUE IS ZERO!!!!!");
    }else{
    	//ROS_INFO_STREAM("THE SEED DEPTH VALUE NOT ZERO!!!!!");
        resultingFrame.at<float>(sY,sX) = 255;                     // add seed to output image.

        // Mark the seed as 1, for the segmentation mask.
    	reached[sY][sX] = 1;
        nPixels++;                                        // increase the pixel counter.

    	// init the queue witht he seed.
        seg_queue.push(std::make_pair(sY,sX));

        /* Loop over the frame, based on the seed adding a
            new pixel to the resultingFrame accordingly*/
    	while(!seg_queue.empty())
    	{
            /* pop values */
    		std::pair<int,int> s = seg_queue.front();
    		int x = s.second;
    		int y = s.first;
            seg_queue.pop();
            /* ... */

            /* The following "if" blocks analise the pixels incorporating them to the
                dst frame if they meet the threshold condition. */

    		// Right pixel
    		if((x + 1 < inputFrame.cols) && (!reached[y][x + 1]) &&
                distanceFunction(inputFrame.at<float>(sY,sX), inputFrame.at<float>(y, x + 1),
                threshold)){
                reached[y][x+1] = true;
                seg_queue.push(std::make_pair(y, x+1));
    			float &pixel = resultingFrame.at<float>(y,x+1);
                pixel = 255;
    			nPixels++;

    		}

    		//Below Pixel
    		if((y+1 < inputFrame.rows) && (!reached[y+1][x]) &&
                distanceFunction(inputFrame.at<float>(sY,sX), inputFrame.at<float>(y+1,x),
                threshold)){
    			reached[y + 1][x] = true;
    			seg_queue.push(std::make_pair(y+1,x));
    			resultingFrame.at<float>(y+1,x) = 255;
    			nPixels++;
    		}

    		//Left Pixel
    		if((x-1 >= 0) && (!reached[y][x-1]) &&
                distanceFunction(inputFrame.at<float>(sY,sX), inputFrame.at<float>(y,x-1),
                threshold)){
    			reached[y][x-1] = true;
    			seg_queue.push(std::make_pair(y,x-1));
    			resultingFrame.at<float>(y,x-1) = 255;
    			nPixels++;
    		}

    		//Above Pixel
    		if((y-1 >= 0) && (!reached[y - 1][x]) &&
                distanceFunction(inputFrame.at<float>(sY,sX), inputFrame.at<float>(y-1,x),
                threshold)){
    			reached[y-1][x] = true;
    			seg_queue.push(std::make_pair(y-1,x));
    			resultingFrame.at<float>(y-1,x) = 255;
    			nPixels++;
    		}

    		//Bottom Right Pixel
    		if((x+1 < inputFrame.cols) && (y+1 < inputFrame.rows) && (!reached[y+1][x+1]) &&
                distanceFunction(inputFrame.at<float>(sY,sX), inputFrame.at<float>(y+1,x+1),
                threshold)){
    			reached[y+1][x+1] = true;
    			seg_queue.push(std::make_pair(y+1,x+1));
    			resultingFrame.at<float>(y+1,x+1) = 255;
    			nPixels++;
    		}

    		//Upper Right Pixel
    		if((x+1 < inputFrame.cols) && (y-1 >= 0) && (!reached[y-1][x+1]) &&
                distanceFunction(inputFrame.at<float>(sY,sX), inputFrame.at<float>(y-1,x+1),
                threshold)){
    			reached[y-1][x+1] = true;
    			seg_queue.push(std::make_pair(y-1,x+1));
    			resultingFrame.at<float>(y-1,x+1) = 255;
    			nPixels++;
    		}

    		//Bottom Left Pixel
    		if((x-1 >= 0) && (y + 1 < inputFrame.rows) && (!reached[y+1][x-1]) &&
                distanceFunction(inputFrame.at<float>(sY,sX), inputFrame.at<float>(y+1,x-1),
                threshold)){
    			reached[y+1][x-1] = true;
    			seg_queue.push(std::make_pair(y+1,x-1));
    			resultingFrame.at<float>(y+1,x-1) = 255;
    			nPixels++;
    		}

    		//Upper left Pixel
    		if((x-1 >= 0) && (y-1 >= 0) && (!reached[y-1][x-1]) &&
                distanceFunction(inputFrame.at<float>(sY,sX), inputFrame.at<float>(y-1,x-1),
                threshold)){
    			reached[y-1][x-1] = true;
    			seg_queue.push(std::make_pair(y-1,x-1));
    			resultingFrame.at<float>(y-1,x-1) = 255;
                nPixels++;
    		}
    	}

        /* FROM THIS POINT ON: Initialization of supporting code for detecting the blob in the
		segmented frame. This is needed for drawing the minimum bounding rectangle
		for the detected blob, thus, enabling the "contraction index" feature
		calculation.*/
        std::vector<std::vector<cv::Point> > contours;
    	std::vector<cv::Vec4i> hierarchy;
        cv::Mat bwImage(inputFrame.size(),inputFrame.type());
        resultingFrame.convertTo(bwImage,CV_8U,255.0/(255-0));
    	cv::findContours(bwImage, contours, hierarchy,
                     CV_RETR_EXTERNAL,
                     CV_CHAIN_APPROX_SIMPLE);


        // Only proceed if at least one contour was found.
    	if (contours.size() > 0){

            cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
            cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(8,8));
            cv::erode(resultingFrame, resultingFrame, erodeElement);
            cv::dilate(resultingFrame, resultingFrame, dilateElement);

            int largest_area=0;
    		int largest_contour_index=0;
    		

    		// find the largest contour in the mask, then use
    		// it to compute the minimum enclosing circle and
    		// centroid
    		for(int i=0; i < contours.size();i++){
    			// iterate through each contour.
                double coutorsArea = cv::contourArea(cv::Mat(contours[i]),false);  //  Find the area of contour
    			if( coutorsArea > largest_area){
    				largest_area=coutorsArea;
    				largest_contour_index=i;   //Store the index of largest contour
    			}
    		}

            //Calculate the minimum bounding box for the detected blob.
            cv::Rect rect = cv::boundingRect(contours[largest_contour_index]);
            cv::Point pt1, pt2;
            pt1.x = rect.x;
            pt1.y = rect.y;
            pt2.x = rect.x + rect.width;
            pt2.y = rect.y + rect.height;

            /* compute the Contraction index feature. It is the difference
            between the bounding rectangle area and the segmented object
            in the segmentation frame, normalized by
            the area of the rectangle. */
            cv::Mat roiSeg = cv::Mat(resultingFrame,rect);
            int roiSegarea = roiSeg.total();
            float ci = float(roiSegarea-nPixels)/float(roiSegarea);
			return ci;
            /* ... */

			////////////////
			// DEPRECATED //
			////////////////

            /*// APPLY COLOR TO THE FRAME
            std::vector<cv::Mat> tSegmentedInColor(3);                   // Used to print a colored
             														  	 //  segmented frame.
            cv::Mat black = cv::Mat::zeros(resultingFrame.rows,
             								resultingFrame.cols,
			 								resultingFrame.type());
            tSegmentedInColor.at(0) = black; 							 //for blue channel
            tSegmentedInColor.at(1) = resultingFrame;   				 //for green channel
            tSegmentedInColor.at(2) = black;  							 //for red channel
            
            cv::merge(tSegmentedInColor, segmentedColorFrame);
            
            //PRINTS THE MARKERS CALCULATED ABOVE.
            cv::circle(segmentedColorFrame, cv::Point(sX,sY),5,
             			cv::Scalar(0,0,255),CV_FILLED, 8,0);
            segmentedTarget = cv::Mat(segmentedColorFrame,rect);
            //cv::circle(segmentedTarget, cv::Point(topPoint,10),5,
            //			cv::Scalar(0,0,255),CV_FILLED, 8,0); // TAGGING THE POINT
            
            // Draws the rect in the segmentedColorFrame image
            cv::rectangle(segmentedColorFrame, pt1, pt2, cv::Scalar(255,255,255), 2,8,0);// the selection white rectangle
            

			cv::putText(segmentedColorFrame,
					std::to_string((int)blobCenter.x),			// The text to be printed, in this case, CI.
					cv::Point(blobCenter.x,blobCenter.y), 		// Coordinates
					cv::FONT_HERSHEY_COMPLEX_SMALL, 			// Font
						0.5, 									// Scale. 2.0 = 2x bigger
						cv::Scalar(255,255,255),				// Color
						1 										// Thickness
						); 										// Anti-alias
            
            // put the CI value to frame
            cv::putText(segmentedColorFrame,
            			std::to_string(ci),				// The text to be printed, in this case, CI.
						cv::Point(rect.x,rect.y-5), 	// Coordinates
						cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
						0.9, 							// Scale. 2.0 = 2x bigger
			 			cv::Scalar(255,255,255),		// Color
						1 								// Thickness
            			); 								// Anti-alias*/
		/////////////////////////////////////////////////////////////////////////////////
		}
    }
}