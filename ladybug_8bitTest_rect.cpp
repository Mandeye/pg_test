//=============================================================================
// Copyright © 2017 FLIR Integrated Imaging Solutions, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of FLIR
// Integrated Imaging Solutions, Inc. ("Confidential Information"). You
// shall not disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with FLIR Integrated Imaging Solutions, Inc. (FLIR).
//
// FLIR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. FLIR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================
//
//=============================================================================
// This example illustrates the simplest procedure of acquiring an image from a
// Ladybug camera.
//
// This program does:
//  - create a context
//  - initialize a camera
//  - start transmission of images
//  - grab an image
//  - color process the grabbed image
//  - save the 6 raw images in BMP files
//  - destroy the context
//
//=============================================================================

#include <stdio.h>
#include <stdlib.h>
#include "ladybug.h"
#include "ladybugstream.h"
#include "ladybuggeom.h"
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#define _HANDLE_ERROR \
    if( error != LADYBUG_OK ) \
   { \
   printf( \
   "Error: Ladybug library reported - %s\n", \
   ::ladybugErrorToString( error ) ); \
   exit( 1 ); \
   } \
   \



const int COLS_COUNT = 2448;
const int ROWS_COUNT = 2048;

typedef struct ladybug_hash_table
{
	float rRow;
	float rCol;
}ladybug_hash_table_t;

// [camera][col][row]
ladybug_hash_table_t hashTableImage[6][ROWS_COUNT][COLS_COUNT];
std::string file_hashTableImage[6];



bool loadHashTableCamera(ladybug_hash_table_t _hashTableImage[][COLS_COUNT], std::string _filename)
{
	std::ifstream f;
	f.open(_filename.c_str());
	if(f.good())
	{
		//int counter = 0;
		while(!f.eof())
		{
			std::string s;
			getline(f,s);

			int dRow, dCol;
			float rRow;
			float rCol;
			sscanf(s.c_str(),"%d %d %f %f", &dRow, &dCol, &rRow, &rCol);

			_hashTableImage[dRow][dCol].rRow = rRow;
			_hashTableImage[dRow][dCol].rCol = rCol;

			//counter++;
		}
		f.close();
		return true;
	}else return false;
}

inline bool _isBlack(cv::Vec3b b)
{
	return b[0]== 0 &&  b[1] == 0 && b[2] == 0;
}


int main( int /* argc */, char* /* argv[] */ )
{

	file_hashTableImage[0] = "rectification0.txt";
	file_hashTableImage[1] = "rectification1.txt";
	file_hashTableImage[2] = "rectification2.txt";
	file_hashTableImage[3] = "rectification3.txt";
	file_hashTableImage[4] = "rectification4.txt";
	file_hashTableImage[5] = "rectification5.txt";


	for(int i = 0 ; i < 6 ; i++)
	{
		if(!loadHashTableCamera(hashTableImage[i], file_hashTableImage[i]))
		{
			printf("Error: !loadHashTableCamera(hushTableImage[%d], %s). Terminating...", i, file_hashTableImage[i].c_str());
			return -1;
		}else
		{
			printf("loadHashTableCamera(hushTableImage[%d], %s) SUCCESS", i, file_hashTableImage[i].c_str());
		}
	}


    // Initialize context.
    LadybugContext context;
    LadybugError error = ::ladybugCreateContext(&context);
    _HANDLE_ERROR;

    // Initialize the first ladybug on the bus.
    printf( "Initializing...\n" );
    error = ::ladybugInitializeFromIndex(context, 0);
    _HANDLE_ERROR;

    // Get camera info
    LadybugCameraInfo caminfo;
    error = ladybugGetCameraInfo(context, &caminfo);
    _HANDLE_ERROR;

    // Start up the camera according to device type and data format
    printf("Starting %s (%u)...\n", caminfo.pszModelName, caminfo.serialHead);

    error = ::ladybugStart(
        context,
        LADYBUG_DATAFORMAT_RAW8);
    _HANDLE_ERROR;




    // Grab a single image.
    printf( "Grabbing image\n" );
    error = LADYBUG_FAILED;
    LadybugImage currentImage;
    for ( int i = 0; i < 10 && error != LADYBUG_OK; i++)
    {
        printf(".");
        error = ::ladybugGrabImage(context, &currentImage);
    }
    for ( int i = 0; i < 10 && error != LADYBUG_OK; i++)
    {
        printf(".");
        error = ::ladybugGrabImage(context, &currentImage);
    }
    printf("\n");
    _HANDLE_ERROR;


    printf("Saving images...\n");



    for (unsigned int uiCamera = 0; uiCamera < LADYBUG_NUM_CAMERAS; uiCamera++)
    {
		std::ostringstream out;

		out << "image" << uiCamera<<".png";

		cv::Size size(currentImage.uiFullCols, currentImage.uiFullRows);
		cv::Mat rawImage(size, CV_8UC1, currentImage.pData + (uiCamera * size.width*size.height));
		cv::Mat imageDebayered;
		cv::cvtColor(rawImage, imageDebayered, cv::COLOR_BayerBG2BGR_EA);
		LadybugImageBorder border = currentImage.imageBorder;

		cv::Mat imageCropped = imageDebayered(cv::Range(border.uiTopRows, size.height-border.uiBottomRows), cv::Range(border.uiLeftCols, size.width-border.uiRightCols));
		cv::transpose(imageCropped, imageCropped);

		cv::Mat ImageRectified = cv::Mat::zeros(imageCropped.size(), CV_8UC4);

		for(int col = 0 ; col < COLS_COUNT; col++)
		{
			for(int row = 0; row < ROWS_COUNT; row++)
			{
				ladybug_hash_table_t ht = hashTableImage[uiCamera][row][col];
				volatile double row_unrect = ht.rRow;
				volatile double col_unrect = ht.rCol;
				if (row_unrect >0  && row_unrect < ROWS_COUNT -1 && col_unrect> 0 && col_unrect < COLS_COUNT -1)
				{
					cv::Vec3b intensity = imageCropped.at<cv::Vec3b>(roundf(col_unrect), roundf(int(row_unrect)));
					cv::Vec4b k;
					k.val[0] =  intensity.val[0];
					k.val[1] =  intensity.val[1];
					k.val[2] =  intensity.val[2];
					k.val[3] =  255;

					ImageRectified.at<cv::Vec4b>(col, 2048-row) = k;
				}

			}
		}
		//cv::Mat ImageRectifiedFilteredl;
		//filter(ImageRectified,ImageRectifiedFilteredl);
		cv::rotate(ImageRectified,ImageRectified, cv::ROTATE_90_COUNTERCLOCKWISE);
		//cv::resize(ImageRectified,ImageRectified, cv::Size(ImageRectified.size().width/2,ImageRectified.size().height/2 ));
		cv::imwrite(out.str(), ImageRectified);
    }

    // Destroy the context
    printf( "Destroying context...\n" );
    error = ::ladybugDestroyContext(&context);
    _HANDLE_ERROR;


    printf( "Done.\n" );

    return 0;
}
