#include "LeapJoint.h"

LeapJoint::LeapJoint()
{
	x = 0.0;
	y = 0.0;
	z = 0.0;
}

LeapJoint::LeapJoint(float newX, float newY, float newZ)
{
	x = newX;
	y = newY;
	z = newZ;
}

float LeapJoint::magnitude()
{
	return sqrt(x * x + y * y + z * z);
}

void LeapJoint::operator=(const LeapJoint& j)
{
	x = j.x;
	y = j.y;
	z = j.z;
}

LeapJoint operator+(const LeapJoint& a, const LeapJoint& b)
{
	LeapJoint result;
	result.x = a.x + b.x;
	result.y = a.y + b.y;
	result.z = a.z + b.z;
	return result;
}

LeapJoint operator-(const LeapJoint& a, const LeapJoint& b)
{
	LeapJoint result;
	result.x = a.x - b.x;
	result.y = a.y - b.y;
	result.z = a.z - b.z;
	return result;
}

LeapJoint operator*(const LeapJoint& j, const float scalar)
{
	LeapJoint result;
	result.x = j.x * scalar;
	result.y = j.y * scalar;
	result.z = j.z * scalar;
	return result;
}


LeapJoint operator/(const LeapJoint& j, const float scalar)
{
	LeapJoint result;
	result.x = j.x / scalar;
	result.y = j.y / scalar;
	result.z = j.z / scalar;
	return result;
}

LeapJoint operator*(const float scalar, const LeapJoint& j)
{
	return j * scalar;
}