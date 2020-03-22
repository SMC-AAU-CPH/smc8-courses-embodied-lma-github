#pragma once

#include <string>
#include "LeapJoint.h"

class IIR4
{
public:
	enum FilterType { LPF, LPD1, LPD2 };
	IIR4() {};
	IIR4(FilterType type, float cutoff);
	LeapJoint process(LeapJoint x);

private:
	LeapJoint x0, x1, x2, x3, x4;
	LeapJoint y0, y1, y2, y3, y4;
	float b0, b1, b2, b3, b4;
	float a0, a1, a2, a3, a4;
	LeapJoint cutoff;

	void makeLPF(float cutoff);
	void makeLPD1(float cutoff);
	void makeLPD2(float cutoff);
};

