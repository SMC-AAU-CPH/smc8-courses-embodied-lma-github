#ifndef leapCallbacks_h
#define leapCallbacks_h

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "LeapC.h"
#include "leapConnection.h"

void initLeapMotion();
float joints[36];
bool frameReady = false;
void* imageBuffer = NULL;
uint32_t imageSize = 0;
int imageWidth;
int imageHeight;
int imageBytesPerPixel;
bool textureChanged = false;
bool imageReady = false;
long int count;

#ifdef __cplusplus
}
#endif

#endif /* leapCallbacks_h */

