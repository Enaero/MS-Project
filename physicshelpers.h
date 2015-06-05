#ifndef __PHYSICSHELPERS_H__
#define __PHYSICSHELPERS_H__

#include <ode/ode.h>

bool displacement_limit(dBodyID body, dVector3 rest_pos, dReal radius, dReal height, const dMatrix3 natural_orientation, const dReal max_angle)
{
	bool limit_reached = false;
	const dReal* current_pos = dBodyGetPosition(body);

	dReal x = current_pos[0] - rest_pos[0];
	dReal y = current_pos[1] - rest_pos[1];
	dReal z = current_pos[2] - rest_pos[2];
	dReal distance = sqrt(x*x + y*y);

	const dReal* velocityPrime = dBodyGetLinearVel(body);
	dVector3 velocity;
	for (int i = 0; i < 3; ++i)
		velocity[i] = velocityPrime[i];

	if (distance > radius)
	{	x = x*radius/distance;
		y = y*radius/distance;

		velocity[0] = 0;
		velocity[1] = 0;
		limit_reached = true;
	}

	if (z > height)
	{
		z = height;
		velocity[2] = 0;
	}

	else if (z < -height)
	{
		z = -height;
		velocity[2] = 0;
	}

	x = rest_pos[0] + x;
	y = rest_pos[1] + y;
	z = rest_pos[2] + z;

	dBodySetPosition(body, x, y, z);
	dBodySetLinearVel(body, velocity[0], velocity[1], velocity[2]);

	dReal angle;
	dVector3 axis;
	calculate_angle_axis(angle, axis, body, natural_orientation);

	if (angle > max_angle)
	{
		//angle = max_angle;
		dBodySetAngularVel(body, 0, 0, 0);

		dMatrix3 R;
		AngleAxistoR(R, angle-max_angle, axis);

		dMatrix3 newRotation;
		multiply_3x3_3x3(newRotation, dBodyGetRotation(body), R);

		dBodySetRotation(body, newRotation);
		dBodySetAngularVel(body, 0, 0, 0);
		limit_reached = true;
	}

	else if (angle < -max_angle)
	{
		//angle = max_angle;
		dBodySetAngularVel(body, 0, 0, 0);

		dMatrix3 R;
		AngleAxistoR(R, angle-max_angle, axis);

		dMatrix3 newRotation;
		multiply_3x3_3x3(newRotation, dBodyGetRotation(body), R);

		dBodySetRotation(body, newRotation);
		dBodySetAngularVel(body, 0, 0, 0);
		limit_reached = true;
	}

	return limit_reached;
}

void translate_body(dBodyID body, dReal dx, dReal dy, dReal dz)
{
	const dReal* old = dBodyGetPosition(body);
	dBodySetPosition(body, old[0] + dx, old[1] + dy, old[2] + dz);
}

void damp_force(dVector3 result, dBodyID body, dReal d)
{
	dVector3 velocity;
	dBodyGetRelPointVel(body, 0, 0, 0, velocity);

	for (int i = 0; i < 3; ++i)
		result[i] = -velocity[i]*d;
}
void damp_angular_vel(dBodyID body, dReal d)
{
	const dReal* angular_vel;
	dVector3 result;
	angular_vel = dBodyGetAngularVel(body);
	dMass m;
	dBodyGetMass(body, &m);

	for (int i = 0; i < 3; ++i)
		result[i] = -angular_vel[i]*d*m.mass;

	dBodyAddTorque(body, result[0], result[1], result[2]);
}

void spring_torque (dVector3 result, dBodyID body, const dMatrix3 natural_orientation, dReal k0, dReal c)
{
	dReal angle;
	calculate_angle_axis(angle, result, body, natural_orientation);
	dReal k = c*angle + k0;

	dReal magnitude = k*angle;

	//cout << angle << ' ' << x << ' ' << y << ' ' << z << endl;

	normalize_4x1(result);
	
	for (int i = 0; i < 3; ++i)
		result[i] = result[i]*magnitude;
}

void stopping_force(dVector3 result, dBodyID body, dVector4 pos_rest, dReal k)
{
	dVector4 pos_diff;
	
	const dReal* pos_current = dBodyGetPosition(body);
	subtract_4x1(pos_diff, pos_current, pos_rest);

	dReal length = magnitude_4x1(pos_diff);
	length = length*length;

	dReal force_magnitude = k*length; 

	dVector3 force;
	for (int i = 0; i < 4; ++i)
		force[i] = pos_diff[i]*-1;

	
	normalize_4x1(force);
	dVector4 force_normal;
	for (int i = 0; i < 4; ++i)
		force_normal[i]= force[i];

	for (int i = 0; i < 4; ++i)
		force[i] *= force_magnitude;

	dVector3 velocity;
	dBodyGetRelPointVel(body, 0, 0, 0, velocity);
	//cout << velocity << ' ';
	dReal relevant_speed = dot_4x1(velocity, force_normal);

	/*dReal damp_magnitude = d*relevant_speed;

	//cout << relevant_speed;
	dVector3 damp_force;
	for (int i = 0; i < 4; ++i)
		damp_force[i] = -force_normal[i]*damp_magnitude;
	
	//cout << damp_force << endl;*/
	for (int i = 0; i < 4; ++i)
		result[i] = force[i];// + damp_force[i];
	/*for (int i = 0; i < 3; ++i)
		result[i] = 0;
	dVector3 current_dist;
	dVector3 after_pos;
	dVector3 after_dist;
	const dReal* current_pos = dBodyGetPosition(body);
	const dReal* vel = dBodyGetLinearVel(body);

	subtract_4x1(current_dist, current_pos, pos_rest);
	add_4x1(after_pos, current_pos, vel);
	subtract_4x1(after_dist, after_pos, pos_rest);

	// if object moving toward rest
	if (magnitude_4x1(after_dist) < magnitude_4x1(current_dist))
		return;

	multiply_4x1_1x1(result, -k, vel);*/
}

void stopping_torque(dVector3 result_force, const dVector4 axis, dBodyID body, dReal k)
{
	dVector3 rel_vel;
	dBodyGetRelPointVel(body, axis[0], axis[1], axis[2], rel_vel);
	
	dVector3 center_vel;
	dBodyGetRelPointVel(body, 0, 0, 0, center_vel);

	subtract_4x1(rel_vel, rel_vel, center_vel);

	dReal magnitude = magnitude_4x1(rel_vel)*k;

	for (int i = 0; i < 3; ++i)
	{
		result_force[i] = -rel_vel[i]*magnitude;
	}
	result_force[3] = 0;

}

void spring_force(dVector3 result, dBodyID body, const dVector4 pos_rest, dReal k)
{
	dVector4 pos_diff;
	
	const dReal* pos_current = dBodyGetPosition(body);
	subtract_4x1(pos_diff, pos_current, pos_rest);
	dReal length = magnitude_4x1(pos_diff);

	dReal force_magnitude = k*length; 

	dVector3 force;
	for (int i = 0; i < 4; ++i)
		force[i] = pos_diff[i]*-1;

	
	normalize_4x1(force);
	dVector4 force_normal;
	for (int i = 0; i < 4; ++i)
		force_normal[i]= force[i];

	for (int i = 0; i < 4; ++i)
		force[i] *= force_magnitude;

	dVector3 velocity;
	dBodyGetRelPointVel(body, 0, 0, 0, velocity);
	//cout << velocity << ' ';
	dReal relevant_speed = dot_4x1(velocity, force_normal);

	/*dReal damp_magnitude = d*relevant_speed;

	//cout << relevant_speed;
	dVector3 damp_force;
	for (int i = 0; i < 4; ++i)
		damp_force[i] = -force_normal[i]*damp_magnitude;
	
	//cout << damp_force << endl;*/
	for (int i = 0; i < 4; ++i)
		result[i] = force[i];// + damp_force[i];

}

#endif