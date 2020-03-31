#pragma once

#include <math.h>

class LeapJoint
{
public:
	LeapJoint();
	LeapJoint(float x, float y, float z);
	//~LeapJoint();

	float x;
	float y;
	float z;

	float magnitude();
	void operator=(const LeapJoint& j);
};

LeapJoint operator*(const float scalar, const LeapJoint& j);
LeapJoint operator+(const LeapJoint& a, const LeapJoint& b);
LeapJoint operator-(const LeapJoint& a, const LeapJoint& b);
LeapJoint operator*(const LeapJoint& j, const float scalar);
LeapJoint operator/(const LeapJoint& j, const float scalar);