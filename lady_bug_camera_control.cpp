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


LadybugContext context;
LadybugError error ;
int gain_slider;
int shutter_slider;
int bright_slider;
void ladybugSetProperrtyPercent(LadybugProperty property, float percent)
{
	 unsigned int plMin,plMax,plDefault;

	//get shutter
	bool present;

	unsigned int plValueA,plValueB;

	bool pbAuto, pbManual;

	error = ::ladybugGetPropertyRange(
		 context,
		 property,
		&present,
		&plMin, &plMax, &plDefault, &pbAuto, &pbManual);
	_HANDLE_ERROR;
	printf ("LADYBUG_SHUTTER\n");
	printf ("	plMin     : %d \n ", plMin);
	printf ("	plMax     : %d \n ", plMax);
	printf ("	plDefault : %d \n ", plDefault);
	printf ("	pbAuto    : %d \n ", pbAuto);
	printf ("	pbManual  : %d \n ", pbManual);

	error = ::ladybugGetProperty(
				context,
				property,
				&plValueA,
				&plValueB,
				&pbAuto );
			_HANDLE_ERROR;
	printf ("	plValueA  : %d \n ", plValueA);
	printf ("	plValueB  : %d \n ", plValueB);
	printf ("	pbAuto    : %d \n ", pbAuto);


	unsigned val =  (1.0*plMin + (plMax- plMin) * percent / 100.0);
	error = ::ladybugSetProperty(
			context,
			property,
			val,
			0,
			false );
	_HANDLE_ERROR;

}
void on_trackbar( int, void* )
{
		ladybugSetProperrtyPercent(LADYBUG_GAIN, gain_slider);
		ladybugSetProperrtyPercent(LADYBUG_SHUTTER, shutter_slider);
		ladybugSetProperrtyPercent(LADYBUG_BRIGHTNESS, bright_slider);





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



			int uiCamera = 0;

		{
			std::ostringstream out;

			out << "image" << uiCamera;

			cv::Size size(currentImage.uiFullCols, currentImage.uiFullRows);
			cv::Mat rawImage(size, CV_8UC1, currentImage.pData + (uiCamera * size.width*size.height));
			cv::Mat imageDebayered;
			cv::cvtColor(rawImage, imageDebayered, cv::COLOR_BayerBG2BGR_EA);
			LadybugImageBorder border = currentImage.imageBorder;

			cv::Mat imageCropped = imageDebayered(cv::Range(border.uiTopRows, size.height-border.uiBottomRows), cv::Range(border.uiLeftCols, size.width-border.uiRightCols));
			cv::Mat small;
			cv::resize(imageCropped, small, cv::Size(0.2*imageCropped.cols, 0.2*imageCropped.rows) );
			cv::imshow("image", small);


		}
}

int main( int /* argc */, char* /* argv[] */ )
{

    // Initialize context.


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

    cv::namedWindow("image", 1);

    cv::createTrackbar( "gain", "image", &gain_slider, 100, on_trackbar);
    cv::createTrackbar( "shutter", "image", &shutter_slider, 100, on_trackbar);
    cv::createTrackbar( "bright_slider", "image", &bright_slider, 100, on_trackbar);


    on_trackbar(0,0);
    cv::waitKey(0);
    // Destroy the context
    printf( "Destroying context...\n" );
    error = ::ladybugDestroyContext(&context);
    _HANDLE_ERROR;


    printf( "Done.\n" );

    return 0;
}
