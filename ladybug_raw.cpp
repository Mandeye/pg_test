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




int main( int /* argc */, char* /* argv[] */ )
{

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
        LADYBUG_DATAFORMAT_RAW16);
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
    printf("\n");
    _HANDLE_ERROR;


    printf("Saving images...\n");

    cv::Size size(currentImage.uiFullCols, 5*currentImage.uiFullRows);
    cv::Mat rawImage(size, CV_16UC1, currentImage.pData);
    cv::imwrite("out.png", rawImage);

	FILE* f = fopen ("out.raw", "wb");
	fwrite (rawImage.data,rawImage.depth(), rawImage.rows * rawImage.cols , f);
	fclose(f);

    for (unsigned int uiCamera = 0; uiCamera < LADYBUG_NUM_CAMERAS; uiCamera++)
    {
		std::ostringstream out;

		out << "image" << uiCamera<<".png";

		cv::Size size(currentImage.uiFullCols, currentImage.uiFullRows);
		cv::Mat rawImage(size, CV_16UC1, currentImage.pData + (uiCamera * size.width*size.height*2));
		cv::Mat imageDebayered;
		cv::cvtColor(rawImage, imageDebayered, cv::COLOR_BayerBG2BGR_EA);
		LadybugImageBorder border = currentImage.imageBorder;

		cv::Mat imageCropped = imageDebayered(cv::Range(border.uiTopRows, size.height-border.uiBottomRows), cv::Range(border.uiLeftCols, size.width-border.uiRightCols));

		//cv::Mat imageCropped = rawImage;
		std::cout << "imageCropped.depth() : " << imageCropped.depth() << "\n";
		std::cout << "imageCropped.rows()  : " << imageCropped.rows << "\n";
		std::cout << "imageCropped.cols()  : " << imageCropped.cols << "\n";
		 cv::imwrite(out.str(), imageCropped);

    }

    // Destroy the context
    printf( "Destroying context...\n" );
    error = ::ladybugDestroyContext(&context);
    _HANDLE_ERROR;


    printf( "Done.\n" );

    return 0;
}
