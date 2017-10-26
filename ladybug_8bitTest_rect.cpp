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
#define _HANDLE_ERROR \
    if( error != LADYBUG_OK ) \
   { \
   printf( \
   "Error: Ladybug library reported - %s\n", \
   ::ladybugErrorToString( error ) ); \
   exit( 1 ); \
   } \
   \

typedef struct ladybug_hash_table
{
	float rRow;
	float rCol;
}ladybug_hash_table_t;

ladybug_hash_table_t hashTableImage[6][2448*2048];
std::string file_hashTableImage[6];



bool loadHashTableCamera(ladybug_hash_table_t *_hashTableImage, std::string _filename)
{
	std::ifstream f;
	f.open(_filename.c_str());
	if(f.good())
	{
		int counter = 0;
		while(!f.eof())
		{
			std::string s;
			getline(f,s);

			int dRow, dCol;
			float rRow;
			float rCol;
			sscanf(s.c_str(),"%d %d %f %f", &dRow, &dCol, &rRow, &rCol);

			_hashTableImage[counter].rRow = rRow;
			_hashTableImage[counter].rCol = rCol;

			counter++;
		}
		f.close();
		return true;
	}else return false;
}

int main( int /* argc */, char* /* argv[] */ )
{

	file_hashTableImage[0] = "hashtable_rectified_CAM0.txt";
	file_hashTableImage[1] = "hashtable_rectified_CAM1.txt";
	file_hashTableImage[2] = "hashtable_rectified_CAM2.txt";
	file_hashTableImage[3] = "hashtable_rectified_CAM3.txt";
	file_hashTableImage[4] = "hashtable_rectified_CAM4.txt";
	file_hashTableImage[5] = "hashtable_rectified_CAM5.txt";


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

		out << "image" << uiCamera<<".ppm";

		cv::Size size(currentImage.uiFullCols, currentImage.uiFullRows);
		cv::Mat rawImage(size, CV_8UC1, currentImage.pData + (uiCamera * size.width*size.height));
		cv::Mat imageDebayered;
		cv::cvtColor(rawImage, imageDebayered, cv::COLOR_BayerBG2BGR_EA);
		LadybugImageBorder border = currentImage.imageBorder;

		cv::Mat imageCropped = imageDebayered(cv::Range(border.uiTopRows, size.height-border.uiBottomRows), cv::Range(border.uiLeftCols, size.width-border.uiRightCols));
		cv::transpose(imageCropped, imageCropped);

		cv::Mat ImageRectified = cv::Mat::zeros(imageCropped.size(), imageCropped.type());

		for(int col = 0 ; col < imageCropped.size().height; col++)
		{
			for(int row = 0; row < imageCropped.size().width; row++)
			{
				cv::Vec3b intensity = imageCropped.at<cv::Vec3b>(col, row);
				ladybug_hash_table_t ht = hashTableImage[uiCamera][row + col * 2048 ];

				ImageRectified.at<cv::Vec3b>(int(ht.rCol), imageCropped.size().width - int(ht.rRow) - 1).val[0] = intensity.val[0];
				ImageRectified.at<cv::Vec3b>(int(ht.rCol), imageCropped.size().width - int(ht.rRow) - 1).val[1] = intensity.val[1];
				ImageRectified.at<cv::Vec3b>(int(ht.rCol), imageCropped.size().width - int(ht.rRow) - 1).val[2] = intensity.val[2];
			}
		}
		cv::rotate(ImageRectified,ImageRectified, cv::ROTATE_90_COUNTERCLOCKWISE);
		cv::resize(ImageRectified,ImageRectified, cv::Size(ImageRectified.size().width/2,ImageRectified.size().height/2 ));
		cv::imwrite(out.str(), ImageRectified);
    }

    // Destroy the context
    printf( "Destroying context...\n" );
    error = ::ladybugDestroyContext(&context);
    _HANDLE_ERROR;


    printf( "Done.\n" );

    return 0;
}
