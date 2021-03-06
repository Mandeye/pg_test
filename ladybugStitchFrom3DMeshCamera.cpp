//=============================================================================
// Copyright � 2017 FLIR Integrated Imaging Solutions, Inc. All Rights Reserved.
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

//=============================================================================
//
// ladybugStitchFrom3DMesh.cpp
// 
// This example shows users how to stitch 6 raw images from the Ladybug without
// using Ladybug SDK.
// Users still need the 3D mesh data produced by the program ladybugOutput3DMesh
// which requires Ladybug SDK.
// This program is useful for users who want to stitch images on the environment
// where Ladybug SDK is not supported.
//
//=============================================================================

#if defined(WIN32) || defined(WIN64)
#include <windows.h>
#include <conio.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include "pgrpnmio.h"
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

// define this if you want to see 3D polygon meshes
//#define DRAW_MESH

//
// global variables
//
LadybugContext context;
bool gInitialized = false;
GLuint gTextures[6];
double gValidTextureWidth;
double gValidTextureHeight;
int gCols, gRows;
double *gTable[6];
bool gAlphamaskAvailable = false;
double gPan = 0.0, gTilt = 0.0;
int gMouseState = GLUT_UP;
int gMouseX, gMouseY;
char *gAlphamaskFilePrefix;
char *gImageFilePrefix;

const int textureW = 1024;
const int textureH = 1024;

unsigned char *ppm_buffer; // store RGB image
unsigned char *pgm_buffer; // store monochrome alpha mask image
int width, height;
unsigned char *texture_buffer;
std::vector<cv::Mat> blendMasks;
//
// helper functions
//
void outputGlError(char* pszLabel) {
	GLenum errorno = glGetError();

	if (errorno != GL_NO_ERROR) {
		printf("%s had error: #(%d) %s\r\n", pszLabel, errorno,
				gluErrorString(errorno));
	}
}

int get_minimum_power_of_two(int in) {
	int i = 1;
	while (i < in) {
		i *= 2;
	}
	return i;
}

bool read_3d_mesh(char *mesh_file_path) {
	FILE *fp = fopen(mesh_file_path, "r");
	if (fp == NULL) {
		printf("Can't read 3D mesh file: %s\n", mesh_file_path);
		return false;
	}

	if (fscanf(fp, "cols %d rows %d\n", &gCols, &gRows) != 2) {
		printf("Can't read cols/rows in 3d mesh file.\n");
		fclose(fp);
		return false;
	}

	for (int c = 0; c < 6; c++) {
		gTable[c] = new double[gCols * gRows * 3];

		for (int iRow = 0; iRow < gRows; iRow++) {
			for (int iCol = 0; iCol < gCols; iCol++) {
				double x, y, z;
				if (fscanf(fp, "%lf, %lf, %lf", &x, &y, &z) != 3) {
					printf("Can't read grid data in 3d mesh file.\n");
					fclose(fp);
					return false;
				}
				gTable[c][(iRow * gCols + iCol) * 3 + 0] = x;
				gTable[c][(iRow * gCols + iCol) * 3 + 1] = y;
				gTable[c][(iRow * gCols + iCol) * 3 + 2] = z;
			}
		}
	}

	fclose(fp);
	return true;
}


void initialize(void) {

	glGenTextures(6, gTextures);
	outputGlError("initialize glGenTextures");



#ifdef DRAW_MESH
		glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
#else
		glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
		glEnable( GL_TEXTURE_2D);
		glShadeModel( GL_FLAT);

		//if (gAlphamaskAvailable) {
			glEnable( GL_BLEND);
			glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		//}
#endif
//	}
}

//
// GLUT handlers
//
void display(void) {
	if (!gInitialized) {
		initialize();
		gInitialized = true;
	}

	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0);
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode( GL_MODELVIEW);
	glLoadIdentity();
	glRotated(-gTilt, 1.0, 0.0, 0.0); // tilt the view
	glRotated(-gPan, 0.0, 0.0, 1.0); // pan the view
	//glRotated(-90.0, 1.0, 0.0, 0.0); // convert Ladybug 3D coordinate system to OpenGL's one

	glMatrixMode( GL_TEXTURE);
	glLoadIdentity();

	glColor3f(1.0f, 1.0f, 1.0f);
	for (int c = 0; c < 6; c++) // for each camera
			{
		glBindTexture( GL_TEXTURE_2D, gTextures[c]);
#ifdef DRAW_MESH
		glColor3f( (c+1)&1, ((c+1)>>1)&1, ((c+1)>>2)&1);
#endif
		for (int iRow = 0; iRow < gRows - 1; iRow++) // for each row
				{
			glBegin( GL_TRIANGLE_STRIP);
			for (int iCol = 0; iCol < gCols; iCol++) // for each column
					{
				double p1 = (double) iCol / (gCols - 1) * gValidTextureWidth;
				double q1 = (double) iRow / (gRows - 1) * gValidTextureHeight;
				double q2 = (double) (iRow + 1.0) / (gRows - 1)
						* gValidTextureHeight;

				int ptr1 = iRow * gCols + iCol;
				double x1 = gTable[c][ptr1 * 3 + 0];
				double y1 = gTable[c][ptr1 * 3 + 1];
				double z1 = gTable[c][ptr1 * 3 + 2];

				int ptr2 = (iRow + 1) * gCols + iCol;
				double x2 = gTable[c][ptr2 * 3 + 0];
				double y2 = gTable[c][ptr2 * 3 + 1];
				double z2 = gTable[c][ptr2 * 3 + 2];

				glTexCoord2d(p1, q1);
				glVertex3d(x1, y1, z1);

				glTexCoord2d(p1, q2);
				glVertex3d(x2, y2, z2);
			}
			glEnd();
		}
	}

	glutSwapBuffers();
}

std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

void idle(void) {
    // Grab a single image.
    printf( "Grabbing image\n" );
    LadybugError error = LADYBUG_FAILED;
    LadybugImage currentImage;
    for ( int i = 0; i < 10 && error != LADYBUG_OK; i++)
    {
        printf(".");
        error = ::ladybugGrabImage(context, &currentImage);
    }
    printf("\n");
    _HANDLE_ERROR;

    for (unsigned int uiCamera = 0; uiCamera < LADYBUG_NUM_CAMERAS; uiCamera++)
    {

		glBindTexture( GL_TEXTURE_2D, gTextures[uiCamera]);
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

		cv::Size size(currentImage.uiFullCols, currentImage.uiFullRows);
		cv::Mat rawImage(size, CV_8UC1, currentImage.pData + (uiCamera * size.width*size.height));


		cv::Mat imageDebayered(size, CV_8UC3);


		cv::cvtColor(rawImage, imageDebayered, cv::COLOR_BayerBG2BGR_EA);
		LadybugImageBorder border = currentImage.imageBorder;

		cv::Mat imageCropped = imageDebayered(cv::Range(border.uiTopRows, size.height-border.uiBottomRows), cv::Range(border.uiLeftCols, size.width-border.uiRightCols));

		cv::Mat imageGL;
		cv::resize(imageCropped, imageGL ,cv::Size(textureW,textureH));

		cv::Mat imageGLAlpha(imageGL.size(), CV_8UC4);

		int from_to[] = { 0,0, 1,1, 2,2, -1,3 };
		cv::mixChannels(&imageGL,1,&imageGLAlpha,1,from_to,4);

		cv::Mat imageGLAlpha2;
		cv::add(blendMasks[uiCamera],imageGLAlpha,imageGLAlpha2);


		gValidTextureWidth  = 1.0;
		gValidTextureHeight = 1.0;
		cv::imwrite("/tmp/dupa1.png",imageGLAlpha2);

		glTexImage2D(
		GL_TEXTURE_2D, 0,
		GL_RGBA8, textureW, textureH, 0,
		GL_BGRA_EXT,
		GL_UNSIGNED_BYTE, imageGLAlpha2.ptr());
		outputGlError("initialize glTexImage2d()");


    }


	glutPostRedisplay();
}

void resize(int w, int h) {
	glViewport(0, 0, w, h);
	glMatrixMode( GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(120.0, (GLfloat) w / (GLfloat) h, 0.1, 100.0);
}

void mouse(int button, int state, int x, int y) {
	gMouseState = state;
	if (gMouseState == GLUT_DOWN) {
		gMouseX = x;
		gMouseY = y;
	}
}

void motion(int x, int y) {
	if (gMouseState == GLUT_DOWN) {
		gPan += (double) (x - gMouseX);
		gMouseX = x;

		gTilt += (double) (y - gMouseY);
		gMouseY = y;
	}
}

int main(int argc, char* argv[]) {

	// Initialize context.

	LadybugError error = ::ladybugCreateContext(&context);
	_HANDLE_ERROR;

	// Initialize the first ladybug on the bus.
	printf("Initializing...\n");
	error = ::ladybugInitializeFromIndex(context, 0);
	_HANDLE_ERROR;

	// Get camera info
	LadybugCameraInfo caminfo;
	error = ladybugGetCameraInfo(context, &caminfo);
	_HANDLE_ERROR;

	// Start up the camera according to device type and data format
	printf("Starting %s (%u)...\n", caminfo.pszModelName, caminfo.serialHead);

	error = ::ladybugStart(context, LADYBUG_DATAFORMAT_RAW8);
	_HANDLE_ERROR;


	// load blends

	std::string blendBaseName="TextureCam";
	for (int i=0; i<6; i++)
	{
		std::stringstream oss;
		oss << blendBaseName <<i<<".pgm";
		cv::Mat blend = cv::imread(oss.str());

		std::cout << type2str(blend.type()) <<"\n";
		cv::Mat blendResized;

		cv::resize(blend, blendResized ,cv::Size(textureW,textureH));
		cv::Mat blendResizedAlphed( blendResized.rows, blendResized.cols, CV_8UC4 );
		int from_to[] = { -1,0, -1,1, -1,2, 1,3 };
		cv::mixChannels(&blendResized,1,&blendResizedAlphed,1,from_to,4);
		blendMasks.push_back(blendResizedAlphed);
		imwrite("a.png",blendResizedAlphed );
	}


	glutInit(&argc, argv);
	glutInitWindowSize(800, 600);
	glutInitDisplayMode( GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	glutCreateWindow("ladybugStitchFrom3DMeshCamera");

	char  *mesh_fn = "mesh.txt";
	if (!read_3d_mesh(mesh_fn)) {
		return 0;
	}

	glutReshapeFunc(resize);
	glutDisplayFunc(display);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutIdleFunc(idle);

	glutMainLoop();

	for (int c = 0; c < 6; c++) {
		delete[] gTable[c];
	}
	// Destroy the context
	printf("Destroying context...\n");
	error = ::ladybugDestroyContext(&context);
	_HANDLE_ERROR;

	delete[] ppm_buffer;
	delete[] texture_buffer;
	if (pgm_buffer != NULL) {
		delete[] pgm_buffer;
	}

	printf("Done.\n");

	return 0;
}
