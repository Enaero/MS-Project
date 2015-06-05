#ifndef __ARMCONTROL_H__
#define __ARMCONTROL_H__

#include "helpers.h"
#include <drawstuff/drawstuff.h>

struct ArmControl
{
	dBodyID base;
	dVector4 restHand;
	dVector4 baseToRestHand;
	dVector4 desiredHand;
	dVector4 currentHand;
	dVector4 currentHandVel;
	dVector4 currentHandAccel;

	dBodyID body;

	ArmControl()
	{}

	ArmControl(dBodyID body, const dReal* restHand, dBodyID base)
	{
		this->body = body;
		set_4x1(this->restHand, restHand);
		this->base = base;
		set_4x1(this->currentHand, restHand);
		set_4x1(this->desiredHand, restHand);
		subtract_4x1(baseToRestHand, restHand, dBodyGetPosition(base));

		set_4x1(currentHandVel, 0, 0, 0, 0);
		set_4x1(currentHandAccel, 0, 0, 0, 0);

	}

	void act()
	{

		dVector4 rotatedDiff;
		const dReal* rotationC = dBodyGetRotation(body);

		multiply_4x4_4x1(rotatedDiff, rotationC, baseToRestHand);

		add_4x1(restHand, dBodyGetPosition(base), rotatedDiff);

		dReal angle;
		dVector3 axis;

		RtoAngleAxis(angle, axis, rotationC);

		angle *= 2; // add angular vel to shoulder, constantly damped and trying to go back to theta = 0. spring and damp

		dMatrix3 rotation;
		dRFromAxisAndAngle(rotation, axis[0], axis[1], axis[2], angle);

		dVector4 rotationResult;
		multiply_4x4_4x1(rotationResult, rotation, baseToRestHand);

		add_4x1(desiredHand, dBodyGetPosition(base), rotationResult);

		dVector4 direction;
		subtract_4x1(direction, desiredHand, currentHand);

		dReal k = 0.01;
		dReal force_size = k*magnitude_4x1(direction);

		normalize_4x1(direction);
		multiply_4x1_1x1(currentHandAccel, force_size, direction);

		dVector4 damp;
		dReal damp_constant = 0.1;
		multiply_4x1_1x1(damp, -damp_constant, currentHandVel);

		add_4x1(currentHandAccel, currentHandAccel, damp);

		add_4x1(currentHandVel, currentHandAccel, currentHandVel);
		add_4x1(currentHand, currentHand, currentHandVel);

	}

	void draw() const
	{
		dsSetColor(1, 0.5, 0.5);
		dsDrawSphereD(currentHand, dBodyGetRotation(body), 0.1);
		dsSetColor(0.5, 1, 0.5);
		dsDrawSphereD(desiredHand, dBodyGetRotation(body), 0.1);
		dsSetColor(0, 0.5, 1);
		dsDrawSphereD(restHand, dBodyGetRotation(body), 0.1);
		dsDrawSphereD(dBodyGetPosition(base), dBodyGetRotation(body), 0.1);
	}
};

#endif