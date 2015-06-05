#ifndef __SKELETON_H__
#define __SKELETON_H__

#include <drawstuff/drawstuff.h>
#include <ode/ode.h>
#include <vector>
#include <map>
#include <iostream>

#include "mathhelpers.h"

using namespace std;

class Skeleton
{
	friend class HybridControl2;
private:
	struct SegmentFamily
	{
		dBodyID parent;
		vector<dBodyID> children;

		SegmentFamily()
		{}
		SegmentFamily(dBodyID parent) : parent(parent), children(vector<dBodyID>())
		{}
	};

	struct SegmentState
	{
		dVector4 pos;
		dMatrix3 rot;

		SegmentState()
		{}
		SegmentState(const dReal* position, const dReal* rotation)
		{
			pos[0] = position[0];
			pos[1] = position[1];
			pos[2] = position[2];
			pos[3] = 1;

			for (int i = 0; i < 3; ++i)
				for (int j = 0; j < 4; ++j)
					rot[i*4+j] = rotation[i*4+j];
		}
	};

	// useful data structures
	vector<dBodyID> segments;
	map<dBodyID, SegmentFamily> tree;
	map<dBodyID, SegmentState> pose;

	// Transformation info
	dReal scaling;

	// body parts
	dBodyID head;
	dBodyID neck;
	dBodyID chest;
	dBodyID abdomen;

	dBodyID lShoulder;
	dBodyID lUpperArm;
	dBodyID lElbow;
	dBodyID lLowerArm;
	dBodyID lHand;

	dBodyID rShoulder;
	dBodyID rUpperArm;
	dBodyID rElbow;
	dBodyID rLowerArm;
	dBodyID rHand;

	dBodyID lHip;
	dBodyID lThigh;
	dBodyID lKnee;
	dBodyID lCalf;
	dBodyID lFoot;

	dBodyID rHip;
	dBodyID rThigh;
	dBodyID rKnee;
	dBodyID rCalf;
	dBodyID rFoot;

	// mass of the segments
	dMass headMass;
	dMass neckMass;
	dMass chestMass;
	dMass abdomenMass;

	dMass lShoulderMass;
	dMass lUpperArmMass;
	dMass lLowerArmMass;
	dMass lHandMass;

	dMass rShoulderMass;
	dMass rUpperArmMass;
	dMass rLowerArmMass;
	dMass rHandMass;

	dMass lHipMass;
	dMass lThighMass;
	dMass lCalfMass;
	dMass lFootMass;

	dMass rHipMass;
	dMass rThighMass;
	dMass rCalfMass;
	dMass rFootMass;

	//Joints
	/*dJointGroupID joints;
	dJointID upperNeckJoint;
	dJointID lowerNeckJoint;
	dJointID midTorsoJoint;

	dJointID lUpperShoulderJoint;
	dJointID lLowerShoulderJoint;
	dJointID lElbowJoint;
	dJointID lWristJoint;
	
	dJointID rUpperShoulderJoint;
	dJointID rLowerShoulderJoint;
	dJointID rElbowJoint;
	dJointID rWristJoint;

	dJointID lUpperHipJoint;
	dJointID lLowerHipJoint;
	dJointID lKneeJoint;
	dJointID lAnkleJoint;

	dJointID rUpperHipJoint;
	dJointID rLowerHipJoint;
	dJointID rKneeJoint;
	dJointID rAnkleJoint;*/

	void rotate_segment_helper(dBodyID segment, dBodyID parent, dReal ax, dReal ay, dReal az, dReal angle);
	void rotate_segment_helper(dBodyID segment, dBodyID parent, const dReal* R);
public:
	enum OrientationCalculationMode
	{
		INERTIA_WEIGHTED,
		MASS_WEIGHTED,
	};
	Skeleton();
	Skeleton(dWorldID world, dReal body_mass);
	void save_pose();
	void draw();
	void translate(dReal x, dReal y, dReal z);
	void shift_CoM(dBodyID pivot, dReal x, dReal y, dReal z);
	void get_CoM(dVector4 result) const;
	void IK_solve(dBodyID upperLimb, const dReal* desiredPosition, const dReal* initialAxis);
	void get_orientation(dVector4 result, OrientationCalculationMode mode = MASS_WEIGHTED) const;
	void rotate_segment(dBodyID segment, const dReal* R);
	void rotate_segment(dBodyID segment, dReal ax, dReal ay, dReal az, dReal angle);
	void reset_segments(dBodyID root);
	void destroy();
};

Skeleton::Skeleton()
{}

Skeleton::Skeleton(dWorldID world, dReal body_mass)
{
	// transformation info
	scaling = 1.0;

	//body parts
	head = dBodyCreate(world);
	neck = dBodyCreate(world);
	chest = dBodyCreate(world);
	abdomen = dBodyCreate(world);

	lShoulder = dBodyCreate(world);
	lUpperArm = dBodyCreate(world);
	lElbow = dBodyCreate(world);
	lLowerArm = dBodyCreate(world);
	lHand = dBodyCreate(world);

	rShoulder = dBodyCreate(world);
	rUpperArm = dBodyCreate(world);
	rElbow = dBodyCreate(world);
	rLowerArm = dBodyCreate(world);
	rHand =	dBodyCreate(world);

	lHip = dBodyCreate(world);
	lThigh = dBodyCreate(world);
	lKnee = dBodyCreate(world);
	lCalf = dBodyCreate(world);
	lFoot = dBodyCreate(world);

	rHip = dBodyCreate(world);
	rThigh = dBodyCreate(world);
	rKnee = dBodyCreate(world);
	rCalf = dBodyCreate(world);
	rFoot = dBodyCreate(world);

	//body parts mass distribution
	dMassSetCapsuleTotal(&headMass, 0.0826*body_mass, 3, 0.1, 0.15); dBodySetMass(head, &headMass);
	dMassSetCylinderTotal(&neckMass, 0.01*body_mass, 3, 0.08, 0.17); dBodySetMass(neck, &neckMass);
	dMassSetBoxTotal(&chestMass, 0.201*body_mass, 0.15, 0.3, 0.5); dBodySetMass(chest, &chestMass);
	dMassSetBoxTotal(&abdomenMass, 0.26*body_mass, 0.15, 0.25, 0.45); dBodySetMass(abdomen, &abdomenMass);

	dMassSetSphereTotal(&lShoulderMass, 0.01*body_mass, 0.1); dBodySetMass(lShoulder, &lShoulderMass);
	dMassSetCylinderTotal(&lUpperArmMass, 0.0325*body_mass, 3, 0.08, 0.35); dBodySetMass(lUpperArm, &lUpperArmMass);
	dMassSetCylinderTotal(&lLowerArmMass, 0.0187*body_mass, 3, 0.08, 0.35); dBodySetMass(lLowerArm, &lLowerArmMass);
	dMassSetSphereTotal(&lHandMass, 0.0065*body_mass, 0.12); dBodySetMass(lHand, &lHandMass);

	dMassSetSphereTotal(&rShoulderMass, 0.01*body_mass, 0.1); dBodySetMass(rShoulder, &rShoulderMass);
	dMassSetCylinderTotal(&rUpperArmMass, 0.0325*body_mass, 3, 0.08, 0.35); dBodySetMass(rUpperArm, &rUpperArmMass);
	dMassSetCylinderTotal(&rLowerArmMass, 0.0187*body_mass, 3, 0.08, 0.35); dBodySetMass(rLowerArm, &rLowerArmMass);
	dMassSetSphereTotal(&rHandMass, 0.0065*body_mass, 0.12); dBodySetMass(rHand, &rHandMass);

	dMassSetSphereTotal(&lHipMass, 0.01*body_mass, 0.1); dBodySetMass(lHip, &lHipMass);
	dMassSetCylinderTotal(&lThighMass, 0.105*body_mass, 3, 0.1, 0.525); dBodySetMass(lThigh, &lThighMass);
	dMassSetCylinderTotal(&lCalfMass, 0.0475*body_mass, 3, 0.1, 0.475); dBodySetMass(lCalf, &lCalfMass);
	dMassSetBoxTotal(&lFootMass, 0.0143*body_mass, .25, .15, .08); dBodySetMass(lFoot, &lFootMass);

	dMassSetSphereTotal(&rHipMass, 0.01*body_mass, 0.1); dBodySetMass(rHip, &rHipMass);
	dMassSetCylinderTotal(&rThighMass, 0.105*body_mass, 3, 0.1, 0.525); dBodySetMass(rThigh, &rThighMass);
	dMassSetCylinderTotal(&rCalfMass, 0.0475*body_mass, 3, 0.1, 0.475); dBodySetMass(rCalf, &rCalfMass);
	dMassSetBoxTotal(&rFootMass, 0.0143*body_mass, .25, .15, .08); dBodySetMass(rFoot, &rFootMass);

	//data structures
	segments.push_back(head);
	segments.push_back(neck);
	segments.push_back(chest);
	segments.push_back(abdomen);
	segments.push_back(lShoulder);
	segments.push_back(lUpperArm);
	segments.push_back(lElbow);
	segments.push_back(lLowerArm);
	segments.push_back(lHand);
	segments.push_back(rShoulder);
	segments.push_back(rUpperArm);
	segments.push_back(rElbow);
	segments.push_back(rLowerArm);
	segments.push_back(rHand);
	segments.push_back(lHip);
	segments.push_back(lThigh);
	segments.push_back(lKnee);
	segments.push_back(lCalf);
	segments.push_back(lFoot);
	segments.push_back(rHip);
	segments.push_back(rThigh);
	segments.push_back(rKnee);
	segments.push_back(rCalf);
	segments.push_back(rFoot);

	tree[abdomen] = SegmentFamily(abdomen);
	tree[abdomen].children.push_back(chest);
	tree[abdomen].children.push_back(lHip);
	tree[abdomen].children.push_back(rHip);

	tree[chest] = SegmentFamily(abdomen);
	tree[chest].children.push_back(neck);
	tree[chest].children.push_back(lShoulder);
	tree[chest].children.push_back(rShoulder);

	tree[neck] = SegmentFamily(chest);
	tree[neck].children.push_back(head);

	tree[head] = SegmentFamily(neck);

	tree[lShoulder] = SegmentFamily(chest);
	tree[lShoulder].children.push_back(lUpperArm);
	tree[lUpperArm] = SegmentFamily(lShoulder);
	tree[lUpperArm].children.push_back(lElbow);
	tree[lElbow] = SegmentFamily(lUpperArm);
	tree[lElbow].children.push_back(lLowerArm);
	tree[lLowerArm] = SegmentFamily(lElbow);
	tree[lLowerArm].children.push_back(lHand);
	tree[lHand] = SegmentFamily(lLowerArm);

	tree[rShoulder] = SegmentFamily(chest);
	tree[rShoulder].children.push_back(rUpperArm);
	tree[rUpperArm] = SegmentFamily(rShoulder);
	tree[rUpperArm].children.push_back(rElbow);
	tree[rElbow] = SegmentFamily(rUpperArm);
	tree[rElbow].children.push_back(rLowerArm);
	tree[rLowerArm] = SegmentFamily(rElbow);
	tree[rLowerArm].children.push_back(rHand);
	tree[rHand] = SegmentFamily(rLowerArm);

	tree[lHip] = SegmentFamily(abdomen);
	tree[lHip].children.push_back(lThigh);
	tree[lThigh] = SegmentFamily(lHip);
	tree[lThigh].children.push_back(lKnee);
	tree[lKnee] = SegmentFamily(lThigh);
	tree[lKnee].children.push_back(lCalf);
	tree[lCalf] = SegmentFamily(lKnee);
	tree[lCalf].children.push_back(lFoot);
	tree[lFoot] = SegmentFamily(lCalf);

	tree[rHip] = SegmentFamily(abdomen);
	tree[rHip].children.push_back(rThigh);
	tree[rThigh] = SegmentFamily(rHip);
	tree[rThigh].children.push_back(rKnee);
	tree[rKnee] = SegmentFamily(rThigh);
	tree[rKnee].children.push_back(rCalf);
	tree[rCalf] = SegmentFamily(rKnee);
	tree[rCalf].children.push_back(rFoot);
	tree[rFoot] = SegmentFamily(rCalf);

	//positioning
	dBodySetPosition(head, 5, 0, 2); 
	dBodySetPosition(neck, 5, 0, 1.88);
	dBodySetPosition(chest, 5, 0, 1.575);
	dBodySetPosition(abdomen, 5, 0, 1.1875);

	dBodySetPosition(lShoulder, 5, -0.18, 1.80);
	dBodySetPosition(lUpperArm, 5, -0.18, 1.625);
	dBodySetPosition(lElbow, 5, -0.18, 1.475);
	dBodySetPosition(lLowerArm, 5, -0.18, 1.275);
	dBodySetPosition(lHand, 5, -0.18, 1.0);

	dBodySetPosition(rShoulder, 5, 0.18, 1.80);
	dBodySetPosition(rUpperArm, 5, 0.18, 1.625);
	dBodySetPosition(rElbow, 5, 0.18, 1.425);
	dBodySetPosition(rLowerArm, 5, 0.18, 1.275);
	dBodySetPosition(rHand, 5, 0.18, 1.0);

	dBodySetPosition(lHip, 5, -0.1, 1.1025);
	dBodySetPosition(lThigh, 5, -0.1, 0.78);
	dBodySetPosition(lKnee, 5, -0.1, 0.53);
	dBodySetPosition(lCalf, 5, -0.1, 0.28);
	dBodySetPosition(lFoot, 5.08, -0.1, 0.0425);

	dBodySetPosition(rHip, 5, 0.1, 1.1025);
	dBodySetPosition(rThigh, 5, 0.1, 0.78);
	dBodySetPosition(rKnee, 5, 0.1, 0.53);
	dBodySetPosition(rCalf, 5, 0.1, 0.28);
	dBodySetPosition(rFoot, 5.08, 0.1, 0.0425);

	//joints
	/*joints = dJointGroupCreate(0);

	upperNeckJoint = dJointCreateBall(world, joints);
	dJointAttach(upperNeckJoint, head, neck);
	dJointSetBallAnchor(upperNeckJoint, 5, 0, 1.90);

	lowerNeckJoint = dJointCreateBall(world, joints);
	dJointAttach(lowerNeckJoint, neck, chest);
	dJointSetBallAnchor(lowerNeckJoint, 5, 0, 1.77);

	midTorsoJoint = dJointCreateBall(world, joints);
	dJointAttach(midTorsoJoint, chest, abdomen);
	dJointSetBallAnchor(midTorsoJoint, 5, 0, 1.35);

	lUpperShoulderJoint = dJointCreateFixed(world, joints);
	dJointAttach(lUpperShoulderJoint, chest, lShoulder);
	dJointSetFixed(lUpperShoulderJoint);

	lLowerShoulderJoint = dJointCreateBall(world, joints);
	dJointAttach(lLowerShoulderJoint, lShoulder, lUpperArm);

	lElbowJoint = dJointCreateHinge(world, joints);
	dJointAttach(lElbowJoint, lUpperArm, lLowerArm);
	dJointSetHingeAxis(lElbowJoint, 0, 1, 0);
	dJointSetHingeAnchor(lElbowJoint, 5, -0.18, 1.45);

	lWristJoint = dJointCreateBall(world, joints);
	dJointAttach(lWristJoint, lLowerArm, lHand);
	dJointSetBallAnchor(lWristJoint, 5, -0.18, 1.1);

	rUpperShoulderJoint = dJointCreateFixed(world, joints);
	dJointAttach(rUpperShoulderJoint, chest, rShoulder);
	dJointSetFixed(rUpperShoulderJoint);

	rLowerShoulderJoint = dJointCreateBall(world, joints);
	dJointAttach(rLowerShoulderJoint, rShoulder, rUpperArm);

	rElbowJoint = dJointCreateHinge(world, joints);
	dJointAttach(rElbowJoint, rUpperArm, rLowerArm);
	dJointSetHingeAxis(rElbowJoint, 0, 1, 0);
	dJointSetHingeAnchor(rElbowJoint, 5, 0.18, 1.45);

	rWristJoint = dJointCreateBall(world, joints);
	dJointAttach(rWristJoint, rLowerArm, rHand);
	dJointSetBallAnchor(rWristJoint, 5, 0.18, 1.1);

	lUpperHipJoint = dJointCreateFixed(world, joints);
	dJointAttach(lUpperHipJoint, abdomen, lHip);
	dJointSetFixed(lUpperHipJoint);

	lLowerHipJoint = dJointCreateBall(world, joints);
	dJointAttach(lLowerHipJoint, lHip, lThigh);

	lKneeJoint = dJointCreateHinge(world, joints);
	dJointAttach(lKneeJoint, lThigh, lCalf);
	dJointSetHingeAxis(lKneeJoint, 0, 1, 0);
	dJointSetHingeAnchor(lKneeJoint, 5, -.1, 0.5175);

	lAnkleJoint = dJointCreateBall(world, joints);
	dJointAttach(lAnkleJoint, lCalf, lFoot);
	dJointSetBallAnchor(lAnkleJoint, 5, -0.1, 0.0425);

	rUpperHipJoint = dJointCreateFixed(world, joints);
	dJointAttach(rUpperHipJoint, abdomen, rHip);
	dJointSetFixed(rUpperHipJoint);

	rLowerHipJoint = dJointCreateBall(world, joints);
	dJointAttach(rLowerHipJoint, rHip, rThigh);

	rKneeJoint = dJointCreateHinge(world, joints);
	dJointAttach(rKneeJoint, rThigh, rCalf);
	dJointSetHingeAxis(rKneeJoint, 0, 1, 0);
	dJointSetHingeAnchor(rKneeJoint, 5, .1, 0.5175);

	rAnkleJoint = dJointCreateBall(world, joints);
	dJointAttach(rAnkleJoint, rCalf, rFoot);
	dJointSetBallAnchor(rAnkleJoint, 5, 0.1, 0.0425);
	*/
	save_pose();
}

void Skeleton::IK_solve(dBodyID upperJoint, const dReal* desiredPosition, const dReal* initialAxis)
{
	//TODO: rewrite
	/*
		first, calculate joint angles using law of cosines
		second, point in the write direction
	*/

	dBodyID upper = tree[upperJoint].children[0];
	dBodyID middle = tree[upper].children[0];
	dBodyID lower = tree[middle].children[0];
	dBodyID end = tree[lower].children[0];

	const dReal* currentPosition = dBodyGetPosition(middle); // treat elbow as end for now
	const dReal* basePosition = dBodyGetPosition(upperJoint);
	dVector3 currentOffset, desiredOffset;

	subtract_4x1(currentOffset, currentPosition, basePosition);
	subtract_4x1(desiredOffset, desiredPosition, basePosition);

	dVector3 axis;
	cross_4x1(axis, currentOffset, desiredOffset);

	dReal cosAngle = dot_4x1(currentOffset, desiredOffset) / (magnitude_4x1(currentOffset)*magnitude_4x1(desiredOffset));
	dReal angle = 0;
	
	if (cosAngle <= -0.9999 || cosAngle >= 0.9999)
		angle = 0;
	else
		angle = acos(cosAngle);

	// point elbow at desired position
	rotate_segment(upper, axis[0], axis[1], axis[2], angle);


	dVector3 upperToMiddle, middleToEnd, middleToDesired;
	subtract_4x1(upperToMiddle, dBodyGetPosition(upperJoint), dBodyGetPosition(middle));
	subtract_4x1(middleToEnd, dBodyGetPosition(end), dBodyGetPosition(middle));
	subtract_4x1(middleToDesired, desiredPosition, dBodyGetPosition(middle));

	cross_4x1(axis, middleToEnd, middleToDesired);

	cosAngle = dot_4x1(middleToEnd, middleToDesired) / (magnitude_4x1(middleToEnd)*magnitude_4x1(middleToDesired));
	if (cosAngle >= 0.9999 || cosAngle <= -0.9999)
		angle = 0;
	else
		angle = acos(cosAngle);

	rotate_segment(lower, axis[0], axis[1], axis[2], angle);

	dReal desiredLength = magnitude_4x1(desiredOffset);
	dReal maximumLength = magnitude_4x1(upperToMiddle) + magnitude_4x1(middleToEnd);

	if ((desiredLength / 2.0) / magnitude_4x1(upperToMiddle) < 1)
	{
		
		//must retract arm
		// calculate angles with the assumption |upper| == |lower|
		// if the assumption is false, can calculate using law of cosines.

		multiply_4x4_4x1(axis, dBodyGetRotation(upper), initialAxis);

		cosAngle = (desiredLength/2.0) / magnitude_4x1(upperToMiddle);
		angle = acos(cosAngle);
		
		rotate_segment(upper, axis[0], axis[1], axis[2], angle);
		rotate_segment(lower, axis[0], axis[1], axis[2], -2*angle);
	}

	/*subtract_4x1(desiredOffset, desiredPosition, basePosition);
	subtract_4x1(currentOffset, dBodyGetPosition(end), basePosition);

	cross_4x1(axis, currentOffset, desiredOffset);
	cosAngle = dot_4x1(currentOffset, desiredOffset) / (magnitude_4x1(currentOffset)*magnitude_4x1(desiredOffset));
	angle = 0;

	if (cosAngle <= -0.9999 || cosAngle >= 0.9999)
		angle = 0;
	else
		angle = acos(cosAngle);

	rotate_segment(upper, axis[0], axis[1], axis[2], angle);*/
}

void Skeleton::save_pose()
{
	for (size_t i = 0; i < segments.size(); ++i)
	{
		const dReal* p = dBodyGetPosition(segments[i]);
		const dReal* r = dBodyGetRotation(segments[i]);
		pose[segments[i]] = SegmentState(p, r);
	}
}
void Skeleton::draw()
{
	const dReal* R;
	const dReal* p;
	dMatrix4 I;
	set_identity_4x4(I);

	p = dBodyGetPosition(head);
	R = dBodyGetRotation(head);
	dsDrawCapsuleD(p, R, 0.15, 0.1);

	p = dBodyGetPosition(neck);
	R = dBodyGetRotation(neck);
	dsDrawCylinderD(p, R, 0.17, 0.08);

	p = dBodyGetPosition(chest);
	R = dBodyGetRotation(chest);
	double chest_sides[3] = {0.15, 0.3, 0.5};
	dsDrawBoxD(p, R, chest_sides);

	p = dBodyGetPosition(abdomen);
	R = dBodyGetRotation(abdomen);
	double abdomen_sides[3] = {0.15, 0.25, 0.35};
	dsDrawBoxD(p, R, abdomen_sides);

	p = dBodyGetPosition(lShoulder);
	R = dBodyGetRotation(lShoulder);
	dsDrawSphereD(p, R, 0.1);

	p = dBodyGetPosition(lUpperArm);
	R = dBodyGetRotation(lUpperArm);
	dsDrawCylinderD(p, R, 0.35, 0.08);

	p = dBodyGetPosition(lLowerArm);
	R = dBodyGetRotation(lLowerArm);
	dsDrawCylinderD(p, R, 0.35, 0.08);

	p = dBodyGetPosition(lHand);
	R = dBodyGetRotation(lHand);
	dsDrawSphereD(p, R, 0.12);

	p = dBodyGetPosition(rShoulder);
	R = dBodyGetRotation(rShoulder);
	dsDrawSphereD(p, R, 0.1);

	p = dBodyGetPosition(rUpperArm);
	R = dBodyGetRotation(rUpperArm);
	dsDrawCylinderD(p, R, 0.35, 0.08);

	p = dBodyGetPosition(rLowerArm);
	R = dBodyGetRotation(rLowerArm);
	dsDrawCylinderD(p, R, 0.35, 0.08);

	p = dBodyGetPosition(rHand);
	R = dBodyGetRotation(rHand);
	dsDrawSphereD(p, R, 0.12);

	p = dBodyGetPosition(lHip);
	R = dBodyGetRotation(lHip);
	dsDrawSphereD(p, R, 0.1);

	p = dBodyGetPosition(lThigh);
	R = dBodyGetRotation(lThigh);
	dsDrawCylinderD(p, R, 0.525, 0.1);

	p = dBodyGetPosition(lCalf);
	R = dBodyGetRotation(lCalf);
	dsDrawCylinderD(p, R, 0.475, 0.1);

	p = dBodyGetPosition(lFoot);
	double lfoot_sides[3] = {0.25, 0.15, 0.08};
	dsDrawBoxD(p, I, lfoot_sides);

	p = dBodyGetPosition(rHip);
	R = dBodyGetRotation(rHip);
	dsDrawSphereD(p, R, 0.1);

	p = dBodyGetPosition(rThigh);
	R = dBodyGetRotation(rThigh);
	dsDrawCylinderD(p, R, 0.525, 0.1);

	p = dBodyGetPosition(rCalf);
	R = dBodyGetRotation(rCalf);
	dsDrawCylinderD(p, R, 0.475, 0.1);

	p = dBodyGetPosition(rFoot);
	double rfoot_sides[3] = {0.25, 0.15, 0.08};
	dsDrawBoxD(p, I, rfoot_sides);
}

void Skeleton::get_CoM(dVector4 result) const
{
	set_4x1(result, 0, 0, 0, 0);
	dReal total_weight = 0.0;
	for (size_t i = 0; i < segments.size(); ++i)
	{
		const dReal* pos = dBodyGetPosition(segments[i]);
		dMass m;
		dBodyGetMass(segments[i], &m);

		dVector4 weighted_pos;
		multiply_4x1_1x1(weighted_pos, m.mass, pos);

		add_4x1(result, result, weighted_pos);
		total_weight += m.mass;
	}

	multiply_4x1_1x1(result, 1.0/total_weight, result);
}

void Skeleton::shift_CoM(dBodyID pivot, dReal x, dReal y, dReal z)
{
	const dReal* pivot_pos = dBodyGetPosition(pivot);
	dVector4 center;
	get_CoM(center);

	dVector3 v;
	subtract_4x1(v, center, pivot_pos);

	dVector4 desired = {x, y, z, 1};
	dVector3 w;
	subtract_4x1(w, desired, pivot_pos);

	v[3] = 0;
	w[3] = 0;

	dVector4 axis;
	cross_4x1(axis, v, w);

	dReal angle = acos(dot_4x1(v, w)/(magnitude_4x1(v)*magnitude_4x1(w)));

	rotate_segment(abdomen, axis[0], axis[1], axis[2], angle);

	dVector3 zero = {1, -1, 0};
	dVector3 t;
	set_4x1(t, v);

	dMatrix4 R;
	set_identity_4x4(R);
	dRFromAxisAndAngle(R, axis[0], axis[1], axis[2], angle);

	multiply_4x4_4x1(t, R, v);

	v[0] += 1;
	v[1] -= 1;
	w[0] += 1;
	w[1] -= 1;
	t[0] += 1;
	t[1] -= 1;

	dsDrawLineD(zero, v);
	dsDrawLineD(zero, w);
	dsSetColor(1, 0, 0);
	dsDrawLineD(zero, t);
}

void Skeleton::get_orientation(dVector4 result, OrientationCalculationMode mode) const
{
	dVector4 center;
	dReal total_I = 0;
	dReal total_mass = 0;
	get_CoM(center);

	set_4x1(result, 0, 0, 0, 0);

	switch (mode)
	{
	case INERTIA_WEIGHTED:
		for (size_t i = 0; i < segments.size(); ++i)
		{
			dVector3 diff;
			subtract_4x1(diff, dBodyGetPosition(segments[i]), center);
			dMass m;
			dBodyGetMass(segments[i], &m);

			dReal I = m.mass*magnitude_4x1(diff)*magnitude_4x1(diff);
			total_I += I;
			dVector4 rotation;
			dReal angle;

			RtoAngleAxis(angle, rotation, dBodyGetRotation(segments[i]));

			normalize_4x1(rotation);
			multiply_4x1_1x1(rotation, 2 * angle, rotation);

			add_4x1(result, result, rotation);
		}

		multiply_4x1_1x1(result, 1.0/total_I, result);
		break;

	case MASS_WEIGHTED:
		for (size_t i = 0; i < segments.size(); ++i)
		{
			dVector3 diff1;
			subtract_4x1(diff1, dBodyGetPosition(segments[i]), center);

			dVector3 diff2;
			subtract_4x1(diff2, pose.at(segments[i]).pos, center);
			
			dVector4 diff_axis;
			cross_4x1(diff_axis, diff2, diff1);

			normalize_4x1(diff_axis);
			multiply_4x1_1x1(diff_axis, angleBetween_4x1(diff1, diff2), diff_axis);

			const dReal*  rot1 = dBodyGetRotation(segments[i]);
			const dReal* rot2 = pose.at(segments[i]).rot;

			dVector4 axis1, axis2;
			dReal angle1, angle2;

			RtoAngleAxis(angle1, axis1, rot1);
			RtoAngleAxis(angle2, axis2, rot2);

			normalize_4x1(axis1);
			multiply_4x1_1x1(axis1, angle1, axis1);

			normalize_4x1(axis2);
			multiply_4x1_1x1(axis2, angle2, axis2);

			dVector4 rot_axis;
			subtract_4x1(rot_axis, axis1, axis2);

			dVector4 total_axis;
			add_4x1(total_axis, rot_axis, diff_axis);

			dMass m;
			dBodyGetMass(segments[i], &m);

			total_mass += m.mass;

			// NOTE: change to total_axis if want to incorporate 
			// the angle differences between position vectors from center of mass
			multiply_4x1_1x1(rot_axis, m.mass, rot_axis);
			add_4x1(result, result, rot_axis);
		}
		multiply_4x1_1x1(result, 1.0 / total_mass, result);
		break;

	default:
		break;
	}
}

void Skeleton::translate(dReal x, dReal y, dReal z)
{
	for (size_t i = 0; i < segments.size(); ++i)
		translate_body(segments[i], x, y, z);
}

void Skeleton::rotate_segment(dBodyID segment, dReal ax, dReal ay, dReal az, dReal angle)
{
	rotate_segment_helper(segment, tree[segment].parent, ax, ay, az, angle);
}

void Skeleton::rotate_segment(dBodyID segment, const dReal* R)
{
	rotate_segment_helper(segment, tree[segment].parent, R);
}

void Skeleton::rotate_segment_helper(dBodyID segment, dBodyID parent, dReal ax, dReal ay, dReal az, dReal angle)
{
	const dReal* parent_pos = dBodyGetPosition(parent);
	const dReal* segment_pos = dBodyGetPosition(segment);

	dVector3 offset;
	subtract_4x1(offset, segment_pos, parent_pos);

	dMatrix4 R;
	dRFromAxisAndAngle(R, ax, ay, az, angle);

	dVector3 rotated_offset;
	multiply_4x4_4x1(rotated_offset, R, offset);

	dVector4 rotated_segment_pos;
	add_4x1(rotated_segment_pos, parent_pos, rotated_offset);

	const dReal* old_R = dBodyGetRotation(segment);
	dMatrix4 new_R;
	multiply_4x4_4x4(new_R, R, old_R);

	dBodySetRotation(segment, new_R);
	dBodySetPosition(segment, rotated_segment_pos[0], rotated_segment_pos[1], rotated_segment_pos[2]);

	for (size_t i = 0; i < tree[segment].children.size(); ++i)
		rotate_segment_helper(tree[segment].children[i], parent, ax, ay, az, angle);
}

void Skeleton::rotate_segment_helper(dBodyID segment, dBodyID parent, const dReal* R)
{
	const dReal* parent_pos = dBodyGetPosition(parent);
	const dReal* segment_pos = dBodyGetPosition(segment);

	dVector3 offset;
	subtract_4x1(offset, segment_pos, parent_pos);

	dVector3 rotated_offset;
	multiply_4x4_4x1(rotated_offset, R, offset);

	dVector4 rotated_segment_pos;
	add_4x1(rotated_segment_pos, parent_pos, rotated_offset);

	const dReal* old_R = dBodyGetRotation(segment);
	dMatrix4 new_R;
	multiply_4x4_4x4(new_R, R, old_R);

	dBodySetRotation(segment, new_R);
	dBodySetPosition(segment, rotated_segment_pos[0], rotated_segment_pos[1], rotated_segment_pos[2]);

	for (size_t i = 0; i < tree[segment].children.size(); ++i)
		rotate_segment_helper(tree[segment].children[i], parent, R);
}

void Skeleton::reset_segments(dBodyID root)
{
	dVector3 old_parent_pos;
	dVector3 old_root_pos;

	const dReal* parent_pos = dBodyGetPosition(tree[root].parent);

	set_4x1(old_parent_pos, pose[tree[root].parent].pos);
	set_4x1(old_root_pos, pose[root].pos);

	dVector3 diff;
	subtract_4x1(diff, old_root_pos, old_parent_pos);
	multiply_4x1_1x1(diff, scaling, diff);

	dVector3 rotated_diff;
	multiply_4x4_4x1(rotated_diff, pose[root].rot, diff);

	dVector4 new_pos;
	add_4x1(new_pos, parent_pos, diff);

	dBodySetPosition(root, new_pos[0], new_pos[1], new_pos[2]);
	dBodySetRotation(root, pose[root].rot);

	for(size_t i = 0; i < tree[root].children.size(); ++i)
		reset_segments(tree[root].children[i]);
}

void Skeleton::destroy()
{
	dBodyDestroy(head);
	dBodyDestroy(neck);
	dBodyDestroy(chest);
	dBodyDestroy(abdomen);

	dBodyDestroy(lShoulder);
	dBodyDestroy(lUpperArm);
	dBodyDestroy(lLowerArm);
	dBodyDestroy(lHand);

	dBodyDestroy(rShoulder);
	dBodyDestroy(rUpperArm);
	dBodyDestroy(rLowerArm);
	dBodyDestroy(rHand);

	dBodyDestroy(lHip);
	dBodyDestroy(lThigh);
	dBodyDestroy(lCalf);
	dBodyDestroy(lFoot);

	dBodyDestroy(rHip);
	dBodyDestroy(rThigh);
	dBodyDestroy(rCalf);
	dBodyDestroy(rFoot);

	//dJointGroupDestroy(joints);
}
#endif