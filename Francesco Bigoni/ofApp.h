#pragma once

#include "ofMain.h"
#include <time.h>
#include <iostream>
#include <cstring>
#include <vector>
#include <algorithm>
#include <cmath>
#include "LeapJoint.h"
#include "IIR4.h"
#include "ofxOsc.h"

extern "C"
{
	// Leap Motion
	extern void DestroyConnection();
	extern float joints[36];
	extern bool frameReady;
	extern void initLeapMotion();
	extern void* imageBuffer;
	extern int imageWidth;
	extern int imageHeight;
	extern int imageBytesPerPixel;
	extern bool imageReady;
}

class ofApp : public ofBaseApp {

public:
	// Standard methods
	void setup();
	void update();
	void draw();
	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	// Motion descriptor calculations
	bool useSmoothing;
	std::vector<LeapJoint> currentJointPositions, jointPos, jointVel, jointAcc, jointJerk;
	std::vector<IIR4> posFilter;
	float qOM;
	float weight1, weight10, weight20, weight50;
	float time1, time10, time20, time50;
	float space10, space20, space50;
	float flow1, flow10, flow20, flow50;
	std::vector<float> weightVector;
	void calculateDescriptors();
	void updateVectors();
	void updateVectorsWithSmoothing();
	void calculateQOM();
	void calculateWeightEffort();
	void calculateTimeEffort();
	void calculateSpaceEffort();
	void calculateFlowEffort();

	// Visuals
	ofImage rawLeapImage;
	unsigned char* prevBuffer;
	void granulateImage();

	// General setup
	int numFrames, numJoints, numCoordinates;
	void resizeVectors();
	//int count;
	bool isFullscreen;
	int frameRate;

	// OSC
	ofxOscSender sender;
	void sendDescriptorsViaOSC();
	bool spatialMode;
};