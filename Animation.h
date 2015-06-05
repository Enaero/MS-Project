#ifndef __POSE_H__
#define __POSE_H__

#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <drawstuff/drawstuff.h>

#include "mathhelpers.h"

#include <map>
#include <string>
#include <vector>
#include <ctime>
using namespace std;

class Pose
{
private:
	struct JointInfo 
	{
		dVector4 limb;
		dVector4 rotated_limb;
		dVector4 global_position;
		dVector4 calculated_position;
		dVector4 rotation;
		dVector4 global_rotation;
		dMatrix3 Jacobian;

		string parent;
		vector<string> children;

		JointInfo();
		void get_end(dVector4& result);
		void draw(dVector4 pos);
	};

	map<string, JointInfo> tree;
	string root_name;
	dVector4 root_pos;
	const aiScene* scene;

	void initialize_help(const aiNode* node, string parent);
	void avg_rtn_help(dVector4 result, string node, unsigned& counter);
	void set_skeleton(string node);
	void set_jacobians(string start, string end);

public:
	void initialize(const aiScene* scene, const aiNode* root, double x = 0, double y = 0, double z = 0);
	void draw_pose(unsigned frame, bool is_animated = true);
	void recursive_draw(string node);
	void set_global_positions(const aiNode* root, dMatrix4 M);
	void set_limbs(string node);
	void set_joint_angles(unsigned frame, int anim_num = 0);
	void rotate_joint(string joint, dReal angle, dReal x, dReal y, dReal z);
	void rotate_joint_pre(string joint, dReal angle, dReal x, dReal y, dReal z);
	void rotate_root_pos(dReal angle, dReal x, dReal y, dReal z);
	void set_root_pos(dReal x, dReal y, dReal z);
	void IK_solve(string start, string end, const dVector4 desired_pos);
	void set_spine_orientation(string base, const dReal* rotation_matrix, string left_shoulder = "", string right_shoulder = "");
	void average_rotation(dVector4 result);
	void center_of_mass(dVector4 result);
	void left_foot_pos(dVector4 result);
	void right_foot_pos(dVector4 result);

	const aiScene* getScene() const;
};

void Pose::set_jacobians(string start, string end)
{
	/* dx/d0_x dx/d0_y dx/d0_z
	   dy/d0_x dy/d0_y ....
	   .
	   .    .
	   .        .
	*/
	string node = end;
	dVector4 old_end_pos;
	dVector4 new_end_pos;
	dVector4 diff;

	while (node != start)
	{
		if (node.size() == 0)
			throw string("Pose::set_jacobians() - Invalid start node");

		// x-axis rotation
		set_4x1(old_end_pos, tree[end].global_position);

		rotate_joint(node, 0.01, 1, 0, 0);
		set_skeleton(node);

		set_4x1(new_end_pos, tree[end].global_position);

		rotate_joint(node, -0.01, 1, 0, 0);
		set_skeleton(node);

		subtract_4x1(diff, new_end_pos, old_end_pos);

		tree[node].Jacobian[0] = diff[0];
		tree[node].Jacobian[3] = diff[1];
		tree[node].Jacobian[6] = diff[2];

		// y-axis rotation
		set_4x1(old_end_pos, tree[end].global_position);

		rotate_joint(node, 0.01, 0, 1, 0);
		set_skeleton(node);

		set_4x1(new_end_pos, tree[end].global_position);

		rotate_joint(node, -0.01, 0, 1, 0);
		set_skeleton(node);

		subtract_4x1(diff, new_end_pos, old_end_pos);

		tree[node].Jacobian[1] = diff[0];
		tree[node].Jacobian[4] = diff[1];
		tree[node].Jacobian[7] = diff[2];

		// z-axis rotation
		set_4x1(old_end_pos, tree[end].global_position);

		rotate_joint(node, 0.01, 0, 0, 1);
		set_skeleton(node);

		set_4x1(new_end_pos, tree[end].global_position);

		rotate_joint(node, -0.01, 0, 0, 1);
		set_skeleton(node);

		subtract_4x1(diff, new_end_pos, old_end_pos);

		tree[node].Jacobian[2] = diff[0];
		tree[node].Jacobian[5] = diff[1];
		tree[node].Jacobian[8] = diff[2];

		node = tree[node].parent;
	}
}

void Pose::IK_solve(string start, string end, const dVector4 desired_pos)
{	
	dVector4 current_leg;
	subtract_4x1(current_leg, tree[end].global_position, tree[start].global_position);

	dVector4 desired_leg;
	subtract_4x1(desired_leg, desired_pos, tree[start].global_position);

	dsDrawLineD(tree[start].global_position, tree[end].global_position);
	dsDrawLineD(tree[start].global_position, desired_pos);

	dVector4 axis, temp;
	cross_4x1(axis, current_leg, desired_leg);
	add_4x1(temp, tree[start].global_position, axis);
	dsDrawLineD(tree[start].global_position, temp);

	dReal angle = acos(dot_4x1(current_leg, desired_leg)/(magnitude_4x1(current_leg)*magnitude_4x1(desired_leg)));

	dMatrix4 R;
	dRFromAxisAndAngle(R, axis[0], axis[1], axis[2], angle);

	normalize_4x1(axis);
	rotate_joint(start, angle, axis[0], axis[1], axis[2]);

	/*string node = end;
	dMatrix4 R;
	dVector4 desired_change;

	set_identity_4x4(R);

	dsSetColor(1, 1, 1);
	dsDrawSphereD(desired_pos, R, 0.2);

	dsSetColor(0, 1, 0);
	dsDrawSphereD(tree[end].global_position, R, 0.2);

	dsSetColor(0, 0, 1);

	subtract_4x1(desired_change, desired_pos, tree[end].global_position);

	do
	{
		node = tree[node].parent;
		set_jacobians(start, end);

		dMatrix3 j_i;
		invert_3x3(j_i, tree[node].Jacobian);

		dVector4 angles;
		multiply_4x4_4x1(angles, j_i, desired_change);

		rotate_joint(node, angles[0], 1, 0, 0);
		rotate_joint(node, angles[1], 0, 1, 0);
		rotate_joint(node, angles[2], 0, 0, 1);

	} while (node != start);

	set_skeleton(start);*/
}

const aiScene* Pose::getScene() const
{
	return scene;
}

void Pose::center_of_mass(dVector4 result)
{
	set_4x1(result, 0, 0, 0, 1);
	for(map<string, JointInfo>::iterator it = tree.begin(); it != tree.end(); ++it)
	{
		for (int i = 0; i < 3; ++i)
			result[i] += it->second.global_position[i];
	}

	for (int i = 0; i < 3; ++i)
		result[i] /= tree.size();
}

void Pose::avg_rtn_help(dVector4 result, string node, unsigned& counter)
{
	if (counter > 3)
		return;
	dVector4 axis_angle;
	dReal qw = tree[node].global_rotation[0];
	dReal qx = tree[node].global_rotation[1];
	dReal qy = tree[node].global_rotation[2];
	dReal qz = tree[node].global_rotation[3];

	dReal angle = 2*acos(qw);

	if (angle == 0)
	{
		axis_angle[0] = 0;
		axis_angle[1] = 0;
		axis_angle[2] = 0;
		axis_angle[3] = 0;
	}
	else
	{
		axis_angle[0] = qx / sin(angle/2.0);
		axis_angle[1] = qy / sin(angle/2.0);
		axis_angle[2] = qz / sin(angle/2.0);
		axis_angle[3] = 0;

		normalize_4x1(axis_angle);
		multiply_4x1_1x1(axis_angle, angle, axis_angle);
	}
	add_4x1(result, result, axis_angle);

	++counter;
	for (size_t i = 0; i < tree[node].children.size(); ++i)
	{
		avg_rtn_help(result, tree[node].children[i], counter);
	}
}

void Pose::average_rotation(dVector4 result)
{
	set_4x1(result, 0, 0, 0, 0);

	unsigned counter = 0;
	avg_rtn_help(result, root_name, counter);

	multiply_4x1_1x1(result, 1.0/counter, result);
}

void Pose::set_limbs(string node)
{
	if(tree[node].parent.length() == 0)
		set_4x1(tree[node].limb, 0, 0, 0, 1);
	else
	{
		subtract_4x1(tree[node].limb, tree[node].global_position, tree[tree[node].parent].global_position);
		//cout << "limb: " << tree[node].limb[0] << ' ' << tree[node].limb[1] << ' ' << tree[node].limb[2] << endl;
		//cout << "pos : " << tree[node].global_position[0] << ' ' << tree[node].global_position[1] << ' ' << tree[node].global_position[2] << endl;
	}
	for (size_t i = 0; i < tree[node].children.size(); ++i)
		set_limbs(tree[node].children[i]);
}

void Pose::initialize(const aiScene* scene, const aiNode* root, double x, double y, double z)
{
	this->scene = scene;
	root_pos[0] = x;
	root_pos[1] = y;
	root_pos[2] = z;
	root_pos[3] = 1;

	root_name = root->mName.C_Str();
	tree[root_name] = JointInfo();
	tree[root_name].parent = "";

	cout << root_name << endl;
	for (unsigned i = 0; i < root->mNumChildren; ++i)
	{
		tree[root_name].children.push_back(root->mChildren[i]->mName.C_Str());
		initialize_help(root->mChildren[i], root_name);
	}

	dMatrix4 I;
	set_identity_4x4(I);
	set_global_positions(root, I);
	set_limbs(root_name);
	cin.get();
}

void Pose::set_spine_orientation(string base, const dReal* rotation_matrix, string left_shoulder, string right_shoulder)
{
	dReal angle;
	dVector4 axis;

	if (left_shoulder.size() > 0 && right_shoulder.size() > 0)
	{
		RtoAngleAxis(angle, axis, rotation_matrix);
		rotate_joint(base, angle/2.0, axis[0], axis[1], axis[2]);
		rotate_joint_pre(left_shoulder, 2*angle, axis[0], axis[1], axis[2]);
		rotate_joint_pre(right_shoulder, 2*angle, axis[0], axis[1], axis[2]);
	}

	else
	{
		RtoAngleAxis(angle, axis, rotation_matrix);

		rotate_joint(base, angle, axis[0], axis[1], axis[2]);
	}
}

void Pose::set_global_positions(const aiNode* root, dMatrix4 M)
{
	string name = root->mName.C_Str();
	dMatrix4 trans;
	dMatrix4 M2;
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
			trans[i*4+j] = root->mTransformation[i][j];
	}

	dVector4 p = {0, 0, 0, 1};
	multiply_4x4_4x4(M2, trans, M);
	multiply_4x4_4x1(p, M2, p);

	//CONSTANT SCALING FACTOR IS HERE! Current value: 75 (15 old)
	multiply_4x1_1x1(tree[name].global_position, 1/75.0, p);

	for (size_t i = 0; i < root->mNumChildren; ++i)
		set_global_positions(root->mChildren[i], M2);
}

void Pose::initialize_help(const aiNode* node, string parent)
{
	string name = node->mName.C_Str();

	cout << name << endl;
	tree[name] = JointInfo();
	tree[name].parent = parent;

	for (unsigned i = 0; i < node->mNumChildren; ++i)
	{
		tree[root_name].children.push_back(node->mChildren[i]->mName.C_Str());
		initialize_help(node->mChildren[i], name);
	}
	
	/*root_name = "root";
	tree["root"] = JointInfo();

	tree["root"].length = 1;
	dQFromAxisAndAngle(tree["root"].rotation, 0, 1, 0, -.2);

	tree["root"].children.push_back("child");
	tree["root"].parent = "";

	tree["child"] = JointInfo();
	tree["child"].length = 1;
	dQFromAxisAndAngle(tree["child"].rotation, 0, 1, 0, -.2);
	tree["child"].parent = "root";

	tree["root"].children.push_back("child2");
	tree["child2"] = JointInfo();
	tree["child2"].length = 1;
	dQFromAxisAndAngle(tree["child2"].rotation, 0, 1, 0, -.4);
	tree["child2"].parent = "root";*/
}

void Pose::set_joint_angles(unsigned frame, int anim_num)
{
	if(!scene->HasAnimations())
		return;
	for (unsigned i = 0; i < scene->mAnimations[anim_num]->mNumChannels; ++i)
	{
		const aiNodeAnim* node = scene->mAnimations[anim_num]->mChannels[i];
		
		if (frame > node->mNumRotationKeys)
			continue;

		tree[node->mNodeName.C_Str()].rotation[0] = node->mRotationKeys[frame].mValue.w;
		tree[node->mNodeName.C_Str()].rotation[1] = node->mRotationKeys[frame].mValue.x;
		tree[node->mNodeName.C_Str()].rotation[2] = node->mRotationKeys[frame].mValue.y;
		tree[node->mNodeName.C_Str()].rotation[3] = node->mRotationKeys[frame].mValue.z;

	}

	// This is for offset, maybe later can use
	/*
	for (unsigned i = 0; i < scene->mAnimations[anim_num]->mNumChannels; ++i)
	{
		const aiNodeAnim* node = scene->mAnimations[anim_num]->mChannels[i];
		
		if (frame > node->mNumPositionKeys || string(node->mNodeName.C_Str()) != root_name)
			continue;

		root_pos[0] = node->mPositionKeys[frame].mValue.x/75;
		root_pos[1] = node->mPositionKeys[frame].mValue.y/75;
		root_pos[2] = node->mPositionKeys[frame].mValue.z/75;

	}*/
}

void Pose::rotate_joint(string joint, dReal angle, dReal x, dReal y, dReal z)
{
	dMatrix4 R;
	dMatrix4 R2;
	dMatrix4 result;

	set_identity_4x4(R);
	set_identity_4x4(R2);
	
	dVector4 q;
	dQFromAxisAndAngle(q, x, y, z, angle);

	dRfromQ(R, tree[joint].rotation);
	dRfromQ(R2, q);

	multiply_4x4_4x4(result, R, R2);

	dQfromR(q, result);

	set_4x1(tree[joint].rotation, q[0], q[1], q[2], q[3]);
}

void Pose::rotate_joint_pre(string joint, dReal angle, dReal x, dReal y, dReal z)
{
	dMatrix4 R;
	dMatrix4 R2;
	dMatrix4 result;

	set_identity_4x4(R);
	set_identity_4x4(R2);
	
	dVector4 q;
	dQFromAxisAndAngle(q, x, y, z, angle);

	dRfromQ(R, tree[joint].rotation);
	dRfromQ(R2, q);

	multiply_4x4_4x4(result, R2, R);

	dQfromR(q, result);

	set_4x1(tree[joint].rotation, q[0], q[1], q[2], q[3]);
}

void Pose::rotate_root_pos(dReal angle, dReal x, dReal y, dReal z)
{
	dMatrix4 R;
	dVector4 temp;
	set_identity_4x4(R);
	dRFromAxisAndAngle(R, x, y, z, angle);

	set_4x1(temp, root_pos[0], root_pos[1], root_pos[2], root_pos[3]);
	multiply_4x4_4x1(root_pos, R, temp);
}

void Pose::set_root_pos(dReal x, dReal y, dReal z)
{
	dMatrix4 R;
	dVector4 temp;
	dRFromAxisAndAngle(R, 1.0, 0, 0, -3.141592/2.0);

	temp[0] = x/2;
	temp[1] = y/2;
	temp[2] = z/2;

	multiply_4x4_4x1(root_pos, R, temp);
}

void Pose::left_foot_pos(dVector4 result)
{
	for (int i = 0; i < 4; ++i)
		result[i] = tree["lFoot"].global_position[i] - tree[root_name].global_position[i];
}

void Pose::right_foot_pos(dVector4 result)
{
	for (int i = 0; i < 4; ++i)
		result[i] = tree["rFoot"].global_position[i]- tree[root_name].global_position[i];
}

void Pose::draw_pose(unsigned frame, bool is_animation)
{
	if (is_animation)
		set_joint_angles(frame);
	rotate_joint_pre(root_name, 3.141592/2.0, 1.0, 0, 0);
	//rotate_joint_pre(root_name, 3.141592/2.0, 0, 0, 1.0);
	rotate_root_pos(3.141592/2.0, 1.0, 0, 0);

	//set_4x1(root_pos, 0, 1, 1, 1);
	//rotate_joint("chest", 3.141592/4.0, 1, 0, 0);
	//rotate_joint("hip", -3.141592/4.0, 1, 0, 0);
	//rotate_joint("rHand", 3.141592/4.0, 0, 1, 0);

	set_skeleton(root_name);
	recursive_draw(root_name);
}

void Pose::recursive_draw(string node)
{
	dReal sides[3] = {0.15, 0.15, 0.15};
	for (int i = 0; i < 3; ++i)
		if (abs(tree[node].limb[i]) > 0.15)
			sides[i] = abs(tree[node].limb[i]);

	dVector4 position;
	if (tree[node].parent.length() > 0)
	{
		add_4x1(position, tree[node].global_position, tree[tree[node].parent].global_position);
		multiply_4x1_1x1(position, 0.5, position);
	}
	else
	{
		for (int i = 0; i < 4; ++i)
			position[i] = tree[node].global_position[i];
	}
	dMatrix3 R;
	dRSetIdentity(R);
	dRfromQ(R, tree[node].global_rotation);

	if (node == "rHand")
		dsSetColor(1, 0, 0);
	else
		dsSetColor(0.5, 0.5, 1);
	dsDrawBoxD(position, R, sides);

	for(size_t i = 0; i < tree[node].children.size(); ++i)
		recursive_draw(tree[node].children[i]);
}

void Pose::set_skeleton(string node)
{
	if(tree[node].parent.size())
	{
		dMatrix4 local_R, global_R, global_R_child;
		set_identity_4x4(local_R);
		set_identity_4x4(global_R);
		set_identity_4x4(global_R_child);

		dRfromQ(local_R, tree[node].rotation);
		dRfromQ(global_R, tree[tree[node].parent].global_rotation);

		multiply_4x4_4x4(global_R_child, global_R, local_R);
		dQfromR(tree[node].global_rotation, global_R_child);

		multiply_4x4_4x1(tree[node].rotated_limb, global_R_child, tree[node].limb);
		add_4x1(tree[node].calculated_position, tree[node].rotated_limb, tree[tree[node].parent].calculated_position);
	}
	else
	{
		set_4x1(tree[node].calculated_position, root_pos[0], root_pos[1], root_pos[2], 1);
		set_4x1(tree[node].global_rotation, tree[node].rotation[0], tree[node].rotation[1], tree[node].rotation[2], tree[node].rotation[3]);
		
		dMatrix4 R;
		set_identity_4x4(R);
		dRfromQ(R, tree[node].global_rotation);
		multiply_4x4_4x1(tree[node].rotated_limb, R, tree[node].limb);
	}

	add_4x1(tree[node].global_position, tree[node].calculated_position, root_pos);

	for (size_t i = 0; i < tree[node].children.size(); ++i)
	{
		set_skeleton(tree[node].children[i]);
	}

}

Pose::JointInfo::JointInfo()
{
	set_4x1(limb, 0, 0, 0, 0);
	set_4x1(rotated_limb, 0, 0, 0, 0);
	set_4x1(global_position, 0, 0, 0, 1);
	set_4x1(calculated_position, 0, 0, 0, 1);
	set_4x1(rotation, 1, 0, 0, 0);
	set_4x1(global_rotation, 1, 0, 0, 0);
}

void Pose::JointInfo::get_end(dVector4& result)
{
	dMatrix4 R;
	dRfromQ(R, global_rotation);

	multiply_4x4_4x1(result, R, limb);
	result[0] = limb[0];
	result[1] = limb[1];
	result[2] = limb[2];
	result[3] = 0;
}

void Pose::JointInfo::draw(dVector4 pos)
{
	dVector4 local_pos;
	dVector4 global_pos;

	get_end(local_pos);
	//multiply_4x1_1x1(local_pos, 0.5, local_pos);
	add_4x1(global_pos, pos, local_pos);

	dMatrix4 R;
	dRfromQ(R, global_rotation);

	dsDrawSphereD(global_pos, global_rotation, 0.1f);
	//dVector3 sides = {magnitude_4x1(limb), 0.1, 0.1};

	//dsDrawBoxD(global_pos, R, sides); 
}

#endif