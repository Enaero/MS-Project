#ifndef __HYBRIDCONTROL_H__
#define __HYBRIDCONTROL_H__

#include "Animation.h"
#include "rigidbodycontrol.h"

class HybridControl
{
private:
	Pose* skeleton;
	RigidBodyControl* rigid_body;

	unsigned frame;
	enum state_t 
	{
		ANIMATION,
		RIGID_BODY
	} state;

public:
	HybridControl();
	HybridControl(Pose* skeleton, RigidBodyControl* rigid_body);

	void act();
	void draw();
};

HybridControl::HybridControl()
	: state(ANIMATION), frame(0), skeleton(NULL), rigid_body(NULL)
{}
HybridControl::HybridControl(Pose* skeleton, RigidBodyControl* rigid_body)
	: state(ANIMATION), frame(0), skeleton(skeleton), rigid_body(rigid_body)
{}

void HybridControl::draw()
{
	dsSetColor(0, 1, 1);
	switch(state)
	{
	case ANIMATION:
		skeleton->draw_pose(frame);
		break;
	case RIGID_BODY:
		skeleton->draw_pose(frame, false);
		break;
	default:
		break;
	}
	rigid_body->draw();
}

void HybridControl::act()
{
	static dVector4 old_lfoot_pos;
	static dVector4 old_rfoot_pos;

	const dReal* body_force = dBodyGetForce(rigid_body->getBody());
	const dReal* body_angular_vel = dBodyGetAngularVel(rigid_body->getBody());
	const dReal* body_linear_vel = dBodyGetLinearVel(rigid_body->getBody());
	const dReal* body_pos = dBodyGetPosition(rigid_body->getBody());

	switch(state)
	{
	case ANIMATION:
		dVector4 test;
		skeleton->average_rotation(test);
		dMatrix4 tempR;
		dMatrix4 R0, R1;
		set_identity_4x4(R1);

		dRFromAxisAndAngle(tempR, test[0], test[1], test[2], magnitude_4x1(test));
		dRFromAxisAndAngle(R0, 1, 0, 0, -3.141592/2.0);
		multiply_4x4_4x4(R1, tempR, R0);
		//dBodySetRotation(rigid_body->getBody(), R1);

		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 4; ++j)
				cout << R1[i*4+j] << ' ';
			cout << endl;
		}
		cout << endl;

		dVector4 center, center_offset;
		set_4x1(center_offset, 0, -1, 0, 0);
		skeleton->center_of_mass(center);
		add_4x1(center, center, center_offset);

		//rigid_body->setRestRotation(R1);
		//rigid_body->setRestPosition(center);

		dVector4 foot1, foot2;
		skeleton->left_foot_pos(foot1);
		skeleton->right_foot_pos(foot2);

		rigid_body->setFoot1Position(foot1);
		rigid_body->setFoot2Position(foot2);

		set_4x1(old_lfoot_pos, rigid_body->getFoot1Position());
		set_4x1(old_rfoot_pos, rigid_body->getFoot2Position());

		skeleton->set_root_pos(body_pos[0], body_pos[1], 1);

		frame = (frame + 2)%(skeleton->getScene()->mAnimations[0]->mChannels[0]->mNumRotationKeys);
		if (frame < 3)
			frame = 3;

		if (magnitude_4x1(body_force) > 0.25)
		{
			skeleton->set_joint_angles(frame);
			state = RIGID_BODY;
		}

		break;
	case RIGID_BODY:
		rigid_body->act();
		skeleton->set_root_pos(body_pos[0], body_pos[1], 1);
		skeleton->set_joint_angles(frame);

		skeleton->IK_solve("lButtock", "lFoot", rigid_body->getFoot1Position());
		skeleton->IK_solve("rButtock", "rFoot", rigid_body->getFoot2Position());
		skeleton->set_spine_orientation("abdomen", dBodyGetRotation(rigid_body->getBody()), "rShldr", "lShldr");

		if (magnitude_4x1(body_angular_vel) < 0.25 && magnitude_4x1(body_linear_vel) < 0.25)
		{
			state = ANIMATION;
		}
		break;
	default:
		break;
	}
}
#endif