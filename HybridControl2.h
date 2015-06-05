#ifndef __HybridControl2_H__
#define __HYBRIDCONTROL2_H__

#include <ode/ode.h>

#include "helpers.h"
#include "Skeleton.h"
#include "rigidbodycontrol.h"
#include "ArmControl.h"

class HybridControl2
{
private:
	Skeleton* skeleton;
	RigidBodyControl* rigid_body;
	ArmControl left_arm_control;
	ArmControl right_arm_control;

public:
	HybridControl2();
	HybridControl2(Skeleton* skeleton, RigidBodyControl* rigid_body);

	void act();
	void draw();
};

HybridControl2::HybridControl2()
	: skeleton(NULL), rigid_body(NULL)
{}
HybridControl2::HybridControl2(Skeleton* skeleton, RigidBodyControl* rigid_body)
	: skeleton(skeleton), rigid_body(rigid_body)
{
	skeleton->rotate_segment(skeleton->lUpperArm, 1, 0, 0, -3.141592/3.0);
	skeleton->rotate_segment(skeleton->rUpperArm, 1, 0, 0, 3.141592/3.0);
	skeleton->rotate_segment(skeleton->lThigh, 1, 0, 0, -3.141592/7.0);
	skeleton->rotate_segment(skeleton->rThigh, 1, 0, 0, 3.141592/7.0);

	skeleton->save_pose();

	left_arm_control = ArmControl(rigid_body->getBody(), dBodyGetPosition(skeleton->lHand), skeleton->neck);
	right_arm_control = ArmControl(rigid_body->getBody(), dBodyGetPosition(skeleton->rHand), skeleton->neck);

}

void HybridControl2::draw()
{
	dsSetColor(0, 1, 1);
	//skeleton->draw();
	rigid_body->draw();
}

void HybridControl2::act()
{
	rigid_body->act();

	dBodyID rigidBodyID = rigid_body->getBody();
	const dReal* rigidBodyPos = dBodyGetPosition(rigidBodyID);
	const dReal* rigidBodyRot = dBodyGetRotation(rigidBodyID);

	dVector4 posDiff;
	const dReal* lFootPos = dBodyGetPosition(skeleton->lFoot);
	const dReal* rFootPos = dBodyGetPosition(skeleton->rFoot);

	//dVector4 lFootOrigPos = { -.27, -.35, 0, 0 };
	//dVector4 rFootOrigPos = { .27, .35, 0, 0 };

	const dReal* lFootOrigPos = rigid_body->getFoot1Position();
	const dReal* rFootOrigPos = rigid_body->getFoot2Position();

	const dReal* abdomenPos = dBodyGetPosition(skeleton->abdomen);
	subtract_4x1(posDiff, rigidBodyPos, abdomenPos);

	skeleton->translate(posDiff[0], posDiff[1], posDiff[2]);

	skeleton->reset_segments(skeleton->abdomen);

	skeleton->rotate_segment(skeleton->abdomen, rigidBodyRot);

	dReal rigidAngle;
	dVector4 rigidAxis;
	calculate_angle_axis(rigidAngle, rigidAxis, rigid_body->getBody(), rigid_body->getRestRotation());

	/*skeleton->rotate_segment(skeleton->lUpperArm, -rigidAxis[0], -rigidAxis[1], -rigidAxis[2], 4*rigidAngle);
	skeleton->rotate_segment(skeleton->rUpperArm, -rigidAxis[0], -rigidAxis[1], -rigidAxis[2], 4*rigidAngle);
	*/

	left_arm_control.act();
	right_arm_control.act();

	dVector4 axis = { 0, -1, 0, 0 };
	dVector4 armAxis = { 0, 1, 0, 0 };

	skeleton->IK_solve(skeleton->lHip, lFootOrigPos, axis);
	skeleton->IK_solve(skeleton->rHip, rFootOrigPos, axis);
	skeleton->IK_solve(skeleton->lShoulder, left_arm_control.currentHand, armAxis);
	skeleton->IK_solve(skeleton->rShoulder, right_arm_control.currentHand, armAxis);

	dVector4 orientation;
	skeleton->get_orientation(orientation, Skeleton::MASS_WEIGHTED);

	normalize_4x1(rigidAxis);
	multiply_4x1_1x1(rigidAxis, -rigidAngle, rigidAxis);

	dVector4 rigidAxisDiff;
	subtract_4x1(rigidAxisDiff, rigidAxis, orientation);

	skeleton->rotate_segment(skeleton->chest, rigidAxisDiff[0], rigidAxisDiff[1], rigidAxisDiff[2], magnitude_4x1(rigidAxisDiff));

	/*if (magnitude_4x1(offsetFromlFoot) < magnitude_4x1(offsetFromrFoot))
	{
		skeleton->shift_CoM(skeleton->rFoot, rigidBodyPos[0], rigidBodyPos[1], rigidBodyPos[2]);
		subtract_4x1(posDiff, rFootOrigPos, rFootPos);
		skeleton->translate(posDiff[0], posDiff[1], posDiff[2]);
		skeleton->IK_solve(skeleton->lHip, lFootOrigPos, axis);
	}
	else
	{
		skeleton->shift_CoM(skeleton->lFoot, rigidBodyPos[0], rigidBodyPos[1], rigidBodyPos[2]);
		subtract_4x1(posDiff, lFootOrigPos, lFootPos);
		skeleton->translate(posDiff[0], posDiff[1], posDiff[2]);
		skeleton->IK_solve(skeleton->rHip, rFootOrigPos, axis);
	}*/

	//left_arm_control.draw();
	//right_arm_control.draw();

	/*dMatrix3 R;
	dRFromAxisAndAngle(R, orientation[0], orientation[1], orientation[2], magnitude_4x1(orientation));
	dVector3 tpos = {0, 3, 1.2};
	dVector4 CoM;
	skeleton->get_CoM(CoM);
	dsDrawCapsuleD(tpos, R, 1.35f, 0.35);

	CoM[0] += 1;
	CoM[1] += 1;
	dsSetColor(0, 1, 0);
	dsDrawSphereD(CoM, dBodyGetRotation(skeleton->abdomen), 0.1);

	dVector4 lfp;
	set_4x1(lfp, lFootPos);
	lfp[0] += 1;
	lfp[1] += 1;
	dsDrawLineD(lfp, CoM);
	dsSetColor(0, 1, 1);*/
}
#endif