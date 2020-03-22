#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() 
{

	// Prepare data acquisition
	numJoints = 12;
	numCoordinates = 36;
	numFrames = 50;
	resizeVectors();
	frameRate = 80;
	initLeapMotion();

	useSmoothing = true;

	for (int i = 0; i < numJoints; i++)
		posFilter[i] = IIR4::IIR4(IIR4::LPF, 0.5);

	ofSetWindowTitle("Unscientific Italians v0.1");
	ofEnableAlphaBlending();
	ofSetupScreen();
	ofBackground(0, 0, 0);
	ofSetFrameRate(frameRate);

	isFullscreen = false;

	ofSetVerticalSync(true);
	ofEnableAlphaBlending();
	ofEnableSmoothing();

	rawLeapImage.allocate(1024, 768, OF_IMAGE_GRAYSCALE);

	ofSetSphereResolution(5);

	ofBackground(0, 0, 0);

	sender.setup("127.0.0.1", 6448);
	spatialMode = false;
}

//--------------------------------------------------------------
void ofApp::update() {

	if (frameReady)
	{
		calculateDescriptors();
		sendDescriptorsViaOSC();
	}

	//if (imageReady)
		//granulateImage();
	
	if (!isFullscreen)
		rawLeapImage.resize(1024, 768);
	else
		rawLeapImage.resize(ofGetWindowWidth(), ofGetWindowHeight());
}

void ofApp::granulateImage() {
	// Runs only once, when uninitialized
	if (prevBuffer == nullptr)
		prevBuffer = (unsigned char*)imageBuffer;

	unsigned char* currentBuffer = (unsigned char*)imageBuffer;
	unsigned char* granulatedBuffer = prevBuffer;
	int bufferLength = imageWidth * imageHeight * imageBytesPerPixel;

	float pitch = ofMap(flow10, 500.0, 180000.0, -4.0, 4.0, true);
	float rate = ofMap(time10, 0.0, 10000.0, 0.0, 10.0, true);
	float grainLength = ofMap(weight10, 0.0, 400.0, 1.8, 0.005, true);
	float nOverlaps = std::round(ofMap(space10, 0.0, 1000.0, 1.0, 10.0, true));

	int drawPositionX = imageHeight - 1;
	int readPositionX = std::round(0.1 * rate * imageHeight);

	for (int i = 0; i < bufferLength; i++) {
		int setPixelIndex = std::round(i * imageWidth / 2 * grainLength + drawPositionX);
		int getPixelIndex = std::round(i * imageWidth / 2 * grainLength * nOverlaps + readPositionX);
		
		if (setPixelIndex > bufferLength)
			setPixelIndex %= bufferLength;

		if (getPixelIndex > bufferLength)
			getPixelIndex %= bufferLength;

		granulatedBuffer[setPixelIndex] = currentBuffer[getPixelIndex];

		drawPositionX -= pitch;
		readPositionX += pitch;

		if (drawPositionX < 0)
			drawPositionX = imageHeight - 1;
	}

	prevBuffer = currentBuffer;

	if (granulatedBuffer != nullptr)
		rawLeapImage.setFromPixels(granulatedBuffer, imageWidth, imageHeight, OF_IMAGE_GRAYSCALE);
	
	if (pitch < 0.0)
		rawLeapImage.mirror(1, 1);
	else
		rawLeapImage.mirror(0, 1);

	imageReady = false;
}

//--------------------------------------------------------------
void ofApp::draw() {
	//if (imageReady)
		//rawLeapImage.draw(0, 0);
	// Uncomment for debugging:
	/*
	string debug = "Flow: " + ofToString(flow20) + ", Time: " + ofToString(time20) + ", Space: " + ofToString(space50) + ", Weight: "
		+ ofToString(weight50) + ", Current sample: " + ofToString(currentSample);

	ofDrawBitmapString(debug, 100, 100);
	*/
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
	if (key == 'f' || key == 'F') 
	{
		ofToggleFullscreen();
		isFullscreen = !isFullscreen;
	}

	else if (key == 'q' || key == 'Q') 
	{
		if (isFullscreen)
			ofToggleFullscreen();

		DestroyConnection();
		ofExit();
	}

	else if (key == 's' || key == 'S') 
	{
		useSmoothing = !useSmoothing;
		if (useSmoothing)
			std::cout << "Smoothing on" << std::endl;
		else
			std::cout << "Smoothing off" << std::endl;
	}

	else if (key == '1')
	{
		spatialMode = false;
		std::cout << "Mode: effort" << std::endl;
	}

	else if (key == '2')
	{
		spatialMode = true;
		std::cout << "Mode: spatial" << std::endl;
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) 
{

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) 
{

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) 
{

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo)  
{

}

void ofApp::calculateDescriptors()
{
	// Fill low-level descriptor vectors
	
	if (!useSmoothing)
		updateVectors();
	
	else
	updateVectorsWithSmoothing();

	// Calculate effort descriptors
	//calculateQOM();
	calculateWeightEffort();
	calculateTimeEffort();
	calculateSpaceEffort();
	calculateFlowEffort();

}

void ofApp::updateVectors() 
{
	float dt = 1.0 / frameRate;

	// shift left on low-level descriptor vectors
	for (int i = 0; i < jointPos.size() - numJoints; i++) 
	{
		jointPos[i] = jointPos[i + numJoints];
		jointVel[i] = jointVel[i + numJoints];
		jointAcc[i] = jointAcc[i + numJoints];
		jointJerk[i] = jointJerk[i + numJoints];
	}

	// store current position values
	int c = 0;

	for (int i = 0; i < numJoints; i++)
	{
		jointPos[jointPos.size() - numJoints + i].x = joints[c++];
		jointPos[jointPos.size() - numJoints + i].y = joints[c++];
		jointPos[jointPos.size() - numJoints + i].z = joints[c++];
	}
		

	// calculate current velocity, acceleration, jerk (lagging 1, 1, 2 frames respectively)
	for (int i = 0; i < numJoints; i++) 
	{
		jointVel[jointVel.size() - numJoints + i] = (jointPos[((numFrames - 1) * numJoints) + i]
			- jointPos[((numFrames - 3) * numJoints) + i]) / (2.0 * dt);
		jointAcc[jointAcc.size() - numJoints + i] = (jointPos[((numFrames - 1) * numJoints) + i]
			- 2.0 * jointPos[((numFrames - 2) * numJoints) + i] + jointPos[((numFrames - 3) * numJoints) + i]) / (dt * dt);
		jointJerk[jointJerk.size() - numJoints + i] = (jointPos[((numFrames - 1) * numJoints) + i]
			- 2.0 * jointPos[((numFrames - 2) * numJoints) + i] + 2.0 * jointPos[((numFrames - 3) * numJoints) + i]
			- jointPos[((numFrames - 4) * numJoints) + i]) / (2.0 * dt * dt * dt);
	}
}

void ofApp::updateVectorsWithSmoothing() 
{
	float dt = 1.0 / frameRate;

	// shift left on low-level descriptor vectors
	for (int i = 0; i < jointPos.size() - numJoints; i++)
	{
		jointPos[i] = jointPos[i + numJoints];
		jointVel[i] = jointVel[i + numJoints];
		jointAcc[i] = jointAcc[i + numJoints];
		jointJerk[i] = jointJerk[i + numJoints];
	}

	// smooth and store current position values

	int c = 0;

	for (int i = 0; i < numJoints; i++)
	{
		currentJointPositions[i].x = joints[c++];
		currentJointPositions[i].y = joints[c++];
		currentJointPositions[i].z = joints[c++];
	}

	for (int i = 0; i < numJoints; i++)
		jointPos[jointPos.size() - numJoints + i] = posFilter[i].process(currentJointPositions[i]);

	// calculate current velocity, acceleration, jerk (lagging 1, 1, 2 frames respectively)
	for (int i = 0; i < numJoints; i++) 
	{
		jointVel[jointVel.size() - numJoints + i] = (jointPos[((numFrames - 1) * numJoints) + i]
			- jointPos[((numFrames - 3) * numJoints) + i]) / (2.0 * dt);
		jointAcc[jointAcc.size() - numJoints + i] = (jointPos[((numFrames - 1) * numJoints) + i]
			- 2.0 * jointPos[((numFrames - 2) * numJoints) + i] + jointPos[((numFrames - 3) * numJoints) + i]) / (dt * dt);
		jointJerk[jointJerk.size() - numJoints + i] = (jointPos[((numFrames - 1) * numJoints) + i]
			- 2.0 * jointPos[((numFrames - 2) * numJoints) + i] + 2.0 * jointPos[((numFrames - 3) * numJoints) + i]
			- jointPos[((numFrames - 4) * numJoints) + i]) / (2.0 * dt * dt * dt);
	}
}

void ofApp::calculateQOM() 
{
	// Calculate quantity of motion

	// Vector of weigthing factors (thumb, index, middle, ring, pinky, wrist for both hands)
	float alpha[12] = { 1, 1, 1, 1, 1, 5, 1, 1, 1, 1, 1, 1 };

	float num = 0;
	float den = 0;

	for (int i = 0; i < numJoints; i++) 
	{
		num += 0.001 * jointVel[(jointVel.size() - numJoints) + i].magnitude() * alpha[i];
		den += alpha[i];
	}

	qOM = num / den;
}

void ofApp::calculateWeightEffort() 
{	
	// Vector of weigthing factors (thumb, index, middle, ring, pinky, palm for both hands)
	float alpha[12] = { 1, 1, 1, 0, 0, 4, 1, 1, 1, 0, 0, 4 };

	float val = 0;

	for (int i = 0; i < numJoints; i++) 
	{
		float mag = 0.001 * jointVel[(jointVel.size() - numJoints) + i].magnitude();
		val += alpha[i] * mag * mag;
	}
	
	// update vector
	for (int i = numFrames - 1; i >= 1; i--)
		weightVector[i - 1] = weightVector[i];

	weightVector[numFrames - 1] = val;

	weight1 = val;
	weight10 = *std::max_element(weightVector.begin() + 39, weightVector.end());
	weight20 = *std::max_element(weightVector.begin() + 29, weightVector.end());
	weight50 = *std::max_element(weightVector.begin(), weightVector.end());
}

void ofApp::calculateTimeEffort() 
{
	int j = 0;

	// Vector of weigthing factors (thumb, index, middle, ring, pinky, palm for both hands)
	float alpha[12] = { 2, 2, 2, 0, 0, 1, 2, 2, 2, 0, 0, 1 };

	float val = 0;

	for (int i = 0; i < numJoints; i++) 
	{
		float mag = 0.001 * jointAcc[(jointAcc.size() - numJoints) + i].magnitude();
		val += alpha[j++] * mag;
	}
	
	j = 0;
	time1 = val;

	for (int i = jointAcc.size() - (10 * numJoints); i < jointAcc.size() - numJoints; i ++) 
	{
			float mag = 0.001 * jointAcc[i].magnitude();
			val += alpha[j++] * mag;
			j %= 12;
	}
	
	j = 0;
	time10 = val / 10.0;

	for (int i = jointAcc.size() - (20 * numJoints); i < jointAcc.size() - (10 * numJoints); i++) {
		float mag = 0.001 * jointAcc[i].magnitude();
		val += alpha[j++] * mag;
		j %= 12;
	}

	j = 0;
	time20 = val / 20.0;

	for (int i = 0; i < jointAcc.size() - (20 * numJoints); i++) {
		float mag = 0.001 * jointAcc[i].magnitude();
		val += alpha[j++] * mag;
		j %= 12;
	}

	time50 = val / 50.0;

	if (!std::isfinite(time10))
		time10 = 0.0;

	if (!std::isfinite(time20))
		time20 = 0.0;

	if (!std::isfinite(time50))
		time50 = 0.0;
}

void ofApp::calculateFlowEffort() 
{
	// Vector of weigthing factors (thumb, index, middle, ring, pinky, palm for both hands)
	float alpha[12] = { 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1 };

	int j = 0;
	float val = 0;

	for (int i = 0; i < numJoints; i++) {
		float mag = 0.001 * jointJerk[(jointJerk.size() - numJoints) + i].magnitude();
		val += alpha[j++] * mag;
	}

	j = 0;
	flow1 = val;

	for (int i = jointJerk.size() - (10 * numJoints); i < jointJerk.size() - numJoints; i++) {
		float mag = 0.001 * jointJerk[i].magnitude();
		val += alpha[j++] * mag;
		j %= 12;
	}

	j = 0;
	flow10 = val / 10.0;

	for (int i = jointJerk.size() - (20 * numJoints); i < jointJerk.size() - (10 * numJoints); i++) {
		float mag = 0.001 * jointJerk[i].magnitude();
		val += alpha[j++] * mag;
		j %= 12;
	}

	j = 0;
	flow20 = val / 20.0;

	for (int i = 0; i < jointJerk.size() - (20 * numJoints); i++) {
		float mag = 0.001 * jointJerk[i].magnitude();
		val += alpha[j++] * mag;
		j %= 12;
	}

	flow50 = val / 50.0;

	if (!std::isfinite(flow10))
		flow10 = 0.0;

	if (!std::isfinite(flow20))
		flow20 = 0.0;

	if (!std::isfinite(flow50))
		flow50 = 0.0;
}

void ofApp::calculateSpaceEffort()
{
	// Vector of weigthing factors (thumb, index, middle, ring, pinky, palm for both hands)
	float alpha[12] = { 1, 5, 1, 0, 0, 1, 1, 5, 1, 0, 0, 1 };

	float num = 0;
	float den = 0;
	space10 = 0;
	space20 = 0;
	space50 = 0;

	for (int i = 0; i < numJoints; i++) {
		LeapJoint diff;
		for (int j = 0; j < 10; j++) {
			diff = jointPos[i + (40 + j) * numJoints] - jointPos[i + (40 + j - 1) * numJoints];
			num += diff.magnitude();
		}

		diff = jointPos[i + 40 * numJoints] - jointPos[i + 49 * numJoints];
		den = diff.magnitude();

		space10 += alpha[i] * num / den;

		for (int j = 0; j < 10; j++) {
			diff = jointPos[i + (30 + j) * numJoints] - jointPos[i + (30 + j - 1) * numJoints];
			num += diff.magnitude();
		}

		diff = jointPos[i + 30 * numJoints] - jointPos[i + 49 * numJoints];
		den = diff.magnitude();

		space20 += alpha[i] * num / den;

		for (int j = 0; j < 30; j++) {
			diff = jointPos[i + (1 + j) * numJoints] - jointPos[i + j * numJoints];
			num += diff.magnitude();
		}

		diff = jointPos[i] - jointPos[i + 49 * numJoints];
		den = diff.magnitude();

		space50 += alpha[i] * num / den;
	}

	if (!std::isfinite(space10))
		space10 = 500.0;

	if (!std::isfinite(space20))
		space20 = 500.0;

	if (!std::isfinite(space50))
		space50 = 500.0;
}

void ofApp::resizeVectors()
{
	currentJointPositions.resize(numJoints);
	jointPos.resize(numJoints * numFrames);
	jointVel.resize(numJoints * numFrames);
	jointAcc.resize(numJoints * numFrames);
	jointJerk.resize(numJoints * numFrames);
	posFilter.resize(numJoints);
	weightVector.resize(numFrames);
}

void ofApp::sendDescriptorsViaOSC()
{
	ofxOscMessage m0;
	m0.setAddress("/wek/inputs");
	
	if (!spatialMode)
	{
		m0.addFloatArg(flow50);
		m0.addFloatArg(time50);
		m0.addFloatArg(weight50);
		m0.addFloatArg(space50);
		m0.addFloatArg(jointPos[jointPos.size() - 1 - 6].magnitude());
		m0.addFloatArg(jointPos[jointPos.size() - 1].magnitude());
	}
	
	else
	{
		m0.addFloatArg(jointPos[jointPos.size() - 1 - 6].x);
		m0.addFloatArg(jointPos[jointPos.size() - 1 - 6].y);
		m0.addFloatArg(jointPos[jointPos.size() - 1 - 6].z);
		m0.addFloatArg(jointPos[jointPos.size() - 1].x);
		m0.addFloatArg(jointPos[jointPos.size() - 1].y);
		m0.addFloatArg(jointPos[jointPos.size() - 1].z);
	}

	sender.sendMessage(m0);

	frameReady = false;
}