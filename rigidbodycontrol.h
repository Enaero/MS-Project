#ifndef __RIGIDBODYCONTROL_H__
#define __RIGIDBODYCONTROL_H__

/* change log (for reverting)
	1. trying to take out displacement_limit or get it to work properly
	2. changing from stepping to predicted_zmp to a desired_step,
	in increments so there are multiple simple steps for hard steps.
	3. limit_height changed to 0
*/
#include <ode/ode.h>

#include "helpers.h"

class RigidBodyControl
{
private:
	dWorldID world;
	dBodyID body;
	dJointID joint;

	dVector4 rest_pos;
	dMatrix3 rest_rot;

	dReal spring_constant;
	dReal linear_damp_constant;
	dReal angular_damp_constant;
	dReal torque_constant;
	dReal torque_constant2;
	dReal step_damp_constant;
	dReal impulse;

	dVector4 zmp; // Zero Moment Point
	dVector4 predicted_zmp;

	dReal zmp_d1;
	dReal zmp_d2;

	dVector4 foot1;
	dVector4 foot2;
	dVector4 old_foot;
	dVector4 temp_desired_foot_pos; //TODO, fix everything, stopped at act yesterday rewrite then implement double desires bip boop
	dVector4 desired_foot_pos;

	dReal stride;
	dReal leg_length;

	dReal stepping_speed;
	dReal step_tolerance; // the largest distance the zmp can be from the closest foot
	dReal limit_radius, limit_height, limit_angle;
	bool limit_reached;

	enum control_state {BALANCING, STEPPING} state;
	dVector4* moving_foot;
	dVector4* standing_foot;

	void predict_zmp(dVector4 pz)
	{
		const dReal* force = dBodyGetForce(body);
		const dReal* position = dBodyGetPosition(body);

		const dReal* cm = position; // position is in center of geom
		const dReal* amom = dBodyGetAngularVel(body); // the mass will cancel out, so no need to calculate it
		const dReal* lmom = dBodyGetLinearVel(body); // the mass will cancel out, so no need to calculate it


		pz[0] = cm[0] - zmp_d1*lmom[0] / (force[2]/10) * cm[2] - zmp_d2*amom[1] / force[2];
		pz[1] = cm[1] - zmp_d1*lmom[1] / (force[2]/10) * cm[2] + zmp_d2*amom[0] / force[2];
		pz[2] = 0;
		pz[3] = 1;
	}

	void calculate_zmp(dVector4 zmp)
	{
		static const dReal* prev_amom = dBodyGetAngularVel(body);
		static const dReal* prev_lmom = dBodyGetLinearVel(body);

		const dReal* force = dBodyGetForce(body);
		const dReal* position = dBodyGetPosition(body);

		const dReal* cm = position; // position is in center of geom
		const dReal* amom = dBodyGetAngularVel(body); // the mass will cancel out, so no need to calculate it
		const dReal* lmom = dBodyGetLinearVel(body); // the mass will cancel out, so no need to calculate it

		dVector3 aaccel, laccel;
		subtract_4x1(aaccel, amom, prev_amom);
		subtract_4x1(laccel, lmom, prev_lmom);

		zmp[0] = cm[0] - laccel[0]/(force[2]/10)*cm[2] - aaccel[1]/force[2];
		zmp[1] = cm[1] - laccel[1]/(force[2]/10)*cm[2] + aaccel[0]/force[2];
		zmp[2] = 0;
		zmp[3] = 1;
	}

	bool decide_to_step()
	{
		// Step should be taken if zmp is outside of support polygon
		// Support polygon is modeled as an ellipse with the feet as the focal points, f1 and f2.
		// Ellipses have this property for a point p ont he ellipse: 
		// |p-f1| + |p-f2| = 2a, where a is the semi-major axis
		// So the zmp is outside the ellipse if |zmp-f1| + |zmp-f2| > 2a

		// First, compute the semi-major axis, a.
		// The step-tolerance is the extent of the ellipse forward.
		
		dVector3 foot_diff; 
		subtract_4x1(foot_diff, foot1, foot2); //f1-f2

		// a^2 = step_tolerance^2 + (foot_diff/2)^2
		dReal a = sqrt(step_tolerance*step_tolerance + dot_4x1(foot_diff, foot_diff) / 4);
		
		// Next, find |zmp-f1| and |zmp-f2|
		dVector3 f1_to_zmp, f2_to_zmp;
		subtract_4x1(f1_to_zmp, foot1, predicted_zmp);
		subtract_4x1(f2_to_zmp, foot2, predicted_zmp);

		
		// Should step if zmp is outside ellipse, so step if |zmp-f1| + |zmp-f2| > 2a
		return magnitude_4x1(f1_to_zmp) + magnitude_4x1(f2_to_zmp) > 2 * a;
	}

public:
	RigidBodyControl()
	{}
	RigidBodyControl(dWorldID world, dBodyID body, dVector4 rest_position, dMatrix3 rest_rotation) : zmp_d1(10), zmp_d2(20)
	{
		this->body = body;
		this->world = world;

		for (char i = 0; i < 3; ++i)
		{
			rest_pos[i] = rest_position[i];
			zmp[i] = rest_position[i];
			predicted_zmp[i] = rest_position[i];
			foot1[i] = rest_position[i];
			foot2[i] = rest_position[i];
		}
		foot1[0] = rest_pos[0] - 1.0;
		foot2[0] = rest_pos[0] + 1.0;
		foot1[2] = 0;
		foot2[2] = 0;

		rest_position[3] = 1;
		zmp[3] = 1;
		predicted_zmp[3] = 1;
		foot1[3] = 1;
		foot2[3] = 1;

		for (char i = 0; i < 12; ++i)
		{ 
			rest_rot[i] = rest_rotation[i];
		}

		spring_constant = 900.0;
		linear_damp_constant = 200;
		torque_constant = 400.0;
		torque_constant2 = 500.0;
		impulse = 950;

		angular_damp_constant = 1.7;
		step_damp_constant = 0;

		limit_height = 0;
		limit_angle = M_PI/8;
		limit_radius = 0.5;
		step_tolerance = 0.1;
		stepping_speed = 2.0/30;
		stride = 1.6;
		leg_length = 1.2;
		moving_foot = &foot1;

		state = BALANCING;
	}

	void draw()
	{
		static int frame_counter = 0;
		if (!limit_reached && frame_counter == 0)
			dsSetColorAlpha(0.7, 0.7, 0.7, 0.5);
		else if (frame_counter != 0)
		{
			frame_counter--;
			dsSetColorAlpha(1.0, 0, 0, 0.5);
		}
		else
		{
			dsSetColorAlpha(1.0, 0, 0, 0.5);
			frame_counter = 15;
		}
		dsDrawCapsuleD(dBodyGetPosition(body), dBodyGetRotation(body), 1.3f, 0.35f);
		
		dsSetColor(1.0, 1.0, 0);
		const dReal* tmp = dBodyGetPosition(body);
		dVector4 pos; pos[0] = tmp[0]; pos[1] = tmp[1]; pos[2] = tmp[2]; pos[3] = 1;

		dReal sides[3] = {0.25, 0.25, 0.25};

		dsSetColor(1.0, 1.0, 0);
		dsDrawSphereD(pos, getRestRotation(), 0.05f);

		
		dsSetColorAlpha(0, 0.3, 1.0, 1);
		//dsDrawBoxD(getFoot1Position(), getRestRotation(), sides);

		dsSetColorAlpha(1.0, 0, 1.0, 1);
		//dsDrawBoxD(getFoot2Position(), getRestRotation(), sides);

		dsSetColor(1.0, 0, 0);
		//dsDrawSphereD(getZeroMomentPoint(), getRestRotation(), 0.05f);

		dsSetColor(0.0, 1.0, 0.0);
		//dsDrawSphereD(getZeroMomentPointPrediction(), getRestRotation(), 0.05f);
	}

	void act()
	{
		calculate_zmp(zmp);
		predict_zmp(predicted_zmp);

		dVector3 diff; // storage vector for subtractions

		switch (state)
		{
		case STEPPING:
			step();
			break;
		case BALANCING:
			balance();
			break;
		default:
			state = BALANCING;
			break;
		}

		dReal height1, height2;
		dVector3 ground_body;
		set_4x1(ground_body, dBodyGetPosition(body)[0], dBodyGetPosition(body)[1], 0, 1);
		subtract_4x1(diff, ground_body, foot1);
		diff[3] = 0;
		dReal square = leg_length*leg_length - dot_4x1(diff, diff);
		if (square >= 0)
			height1 = sqrt(square);
		else
			height1 = leg_length;

		subtract_4x1(diff, ground_body, foot2);
		diff[3] = 0;
		square = leg_length*leg_length - dot_4x1(diff, diff);
		if (square >= 0)
			height2 = sqrt(square);
		else
			height2 = leg_length;

		rest_pos[2] = height1 < height2 ? height1 : height2;

		limit_reached = displacement_limit(body, rest_pos, limit_radius, limit_height, rest_rot, limit_angle);
	}

	void step()
	{
		static dReal frame_count = 0;
		static bool stepping = true;

		balance_helper();
		dVector3 diff;
		subtract_4x1(diff, *standing_foot, *moving_foot);

		dVector3 zmp_diff_moving;
		subtract_4x1(zmp_diff_moving, predicted_zmp, *moving_foot);

		dVector3 zmp_diff_standing;
		subtract_4x1(zmp_diff_standing, predicted_zmp, *standing_foot);

		dReal dist = magnitude_4x1(diff);

		dVector3 step_vector;
		subtract_4x1(step_vector, predicted_zmp, old_foot);

		/*
		//TODO: try rotation
		// begin check foot trajectory collision
		dVector3 up = { 0, 0, 1 };
		dVector3 perp;
		cross_4x1(perp, step_vector, up);

		dVector3 collision_vector;
		subtract_4x1(collision_vector, *standing_foot, old_foot);

		dReal perp_dist = dot_4x1(collision_vector, perp)/magnitude_4x1(perp);

		std::cout << perp_dist << std::endl;
		if (abs(perp_dist) < 0.1)
		{
			//normalize_4x1(perp);
			//multiply_4x1_1x1(perp, -0.5, perp);
			//add_4x1(step_vector, step_vector, perp);

			dMatrix4 R;
			set_identity_4x4(R);
			
			if (moving_foot == &foot2)
				dRFromAxisAndAngle(R, 0, 0, 1, -0.1);
			else
				dRFromAxisAndAngle(R, 0, 0, 1, 0.1);
			
			dVector4 new_diff;
			multiply_4x4_4x1(new_diff, R, diff);

			dVector4 new_moving;
			subtract_4x1(new_moving, *standing_foot, new_diff);
			set_4x1(*moving_foot, new_moving);

			const dReal* old_R = dBodyGetRotation(body);
			dMatrix4 new_R;
			multiply_4x4_4x4(new_R, old_R, R);
			//dBodySetRotation(body, new_R);
		}
		// end foot trajectory collision
		*/
		if (!stepping)
		{
			frame_count = 0;
			state = BALANCING;
			stepping = true;
			//const dReal* vel = dBodyGetLinearVel(body);
			//const dReal* avel = dBodyGetAngularVel(body);

			//dBodySetLinearVel(body, vel[0]*step_damp_constant, vel[1]*step_damp_constant, vel[2]);
			//dBodySetAngularVel(body, avel[0], avel[1], avel[2]);
		}

		else
		{
			dReal total_frames = 17;
			dReal accel_frames_ratio = 0.3;

			dReal max_speed = magnitude_4x1(step_vector)*(1.0 - accel_frames_ratio)/(total_frames-2*accel_frames_ratio*total_frames);

			dReal accel = max_speed/(accel_frames_ratio*total_frames);

			dReal speed = 0.0;

			if (frame_count < accel_frames_ratio*total_frames)
				speed = accel*frame_count;
			else if (frame_count >= accel_frames_ratio*total_frames && frame_count < total_frames - accel_frames_ratio*total_frames)
				speed = max_speed;
			else
				speed = max_speed - accel*(frame_count - (total_frames - total_frames*accel_frames_ratio));

			for (char i = 0; i < 3; ++i)
				moving_foot[0][i] += step_vector[i]/magnitude_4x1(step_vector)*speed;

			rest_pos[0] = (*standing_foot)[0];
			rest_pos[1] = (*standing_foot)[1];

			frame_count += 1;
			if (frame_count == total_frames || magnitude_4x1(diff) > stride && magnitude_4x1(zmp_diff_moving) < magnitude_4x1(zmp_diff_standing))
			{
				stepping = false;
				balance_helper(true);
				rest_pos[0] = (foot1[0] + foot2[0]) / 2.0;
				rest_pos[1] = (foot1[1] + foot2[1]) / 2.0;
			}
		}
	}

	void balance_helper(bool linear_control = true)
	{
		dVector3 spr_force;
		dVector3 spr_torque;
		dVector3 dmp_force;
		dVector3 stop_force;

		dReal stop_constant = 1;
		if(linear_control)
		{
			spring_force(spr_force, body, rest_pos, spring_constant);
			stopping_force(stop_force, body, rest_pos, stop_constant);
			damp_force(dmp_force, body, linear_damp_constant);

			dBodyAddForce(body, spr_force[0], spr_force[1], spr_force[2]);
			dBodyAddForce(body, dmp_force[0], dmp_force[1], dmp_force[2]);
			dBodyAddForce(body, stop_force[0], stop_force[1], stop_force[2]);

			spring_torque(spr_torque, body, rest_rot, torque_constant, torque_constant2);
			//dBodySetAngularDamping(body, angular_damp_constant);
			damp_angular_vel(body, angular_damp_constant);
			dBodyAddTorque(body, spr_torque[0], spr_torque[1], spr_torque[2]);
		}
		else
		{
			spring_torque(spr_torque, body, rest_rot, torque_constant, torque_constant2);
			//dBodySetAngularDamping(body, angular_damp_constant);
			damp_angular_vel(body, angular_damp_constant);
			dBodyAddTorque(body, spr_torque[0], spr_torque[1], spr_torque[2]);
		}
	}

	void balance()
	{
		static int frame_count = 0;

		balance_helper();

		const dReal* force = dBodyGetForce(body);
		const dReal* position = dBodyGetPosition(body);

		
		if (force[2] < -0.1)
		{
			if (true)//!decide_to_step())
			{
				state = BALANCING;
			}
			
			else if (frame_count++ > 4 /*used to be 10*/)
			{
				frame_count = 0;
				state = STEPPING;

				const dReal* tmp = dBodyGetPosition(body);
				const dReal* vel = dBodyGetLinearVel(body);

				dReal cmp[4];
				cmp[0] = tmp[0]; cmp[1] = tmp[1]; cmp[2] = 0; cmp[3] = 1;

				dReal dcf1x = cmp[0] - foot1[0];
				dReal dcf1y = cmp[1] - foot1[1];

				dReal dcf2x = cmp[0] - foot2[0];
				dReal dcf2y = cmp[1] - foot2[1];

				dReal dcf1 = dcf1x*dcf1x + dcf1y*dcf1y;
				dReal dcf2 = dcf2x*dcf2x + dcf2y*dcf2y;

				dReal future_cmp[4];
				future_cmp[0] = tmp[0] + vel[0]*1.0; cmp[1] = tmp[1] + vel[1]*1.0; cmp[2] = 0; cmp[3] = 1;

				dReal future_dcf1x = future_cmp[0] - foot1[0];
				dReal future_dcf1y = future_cmp[1] - foot1[1];

				dReal future_dcf2x = future_cmp[0] - foot1[0];
				dReal future_dcf2y = future_cmp[1] - foot2[1];

				dReal future_dcf1 = dcf1x*dcf1x + dcf1y*dcf1y;
				dReal future_dcf2 = dcf2x*dcf2x + dcf2y*dcf2y;

				dVector3 foot_diff;
				subtract_4x1(foot_diff, foot1, foot2);


				if (magnitude_4x1(foot_diff) > stride*0.8)
				{
					dVector3 diff1, diff2;
					subtract_4x1(diff1, predicted_zmp, foot1);
					subtract_4x1(diff2, predicted_zmp, foot2);

					bool check = magnitude_4x1(diff1) > magnitude_4x1(diff2);
					moving_foot = check ? &foot1 : &foot2;
					standing_foot = check ? &foot2 : &foot1;
				}
				else if (dcf1/dcf2 > 3.0 || dcf2/dcf1 > 3.0)
				{
					moving_foot = dcf1 > dcf2 ? &foot1 : &foot2;
					standing_foot = dcf1 > dcf2 ? &foot2 : &foot1;
				}
				else
				{
					moving_foot = future_dcf1 < dcf2 ? &foot1 : &foot2;
					standing_foot = future_dcf1 < dcf2 ? &foot2 : &foot1;
				}

				dVector3 center_to_foot1;
				dVector3 center_to_foot2;

				subtract_4x1(center_to_foot1, foot1, dBodyGetPosition(body));
				subtract_4x1(center_to_foot2, foot2, dBodyGetPosition(body));
				center_to_foot1[2] = 0;
				center_to_foot2[2] = 0;
				

				if (magnitude_4x1(center_to_foot1) < magnitude_4x1(center_to_foot2))
				{
					moving_foot = &foot2;
					standing_foot = &foot1;
				}
				else
				{
					moving_foot = &foot1;
					standing_foot = &foot2;
				}


				/*dVector4 trajectory, foot_trajectory1, foot_trajectory2;
				subtract_4x1(trajectory, predicted_zmp, dBodyGetPosition(body));
				subtract_4x1(foot_trajectory1, predicted_zmp, *standing_foot);
				subtract_4x1(foot_trajectory2, predicted_zmp, *moving_foot);

				if (angleBetween_4x1(foot_trajectory1, trajectory) < angleBetween_4x1(foot_trajectory2, trajectory))
				{
					swap(standing_foot, moving_foot);
				}
				
				dVector4 trajectory;
				subtract_4x1(foot_diff, *standing_foot, *moving_foot);
				subtract_4x1(trajectory, predicted_zmp, *moving_foot);
				
				if (angleBetween_4x1(foot_diff, trajectory) < 0.2)
					swap(moving_foot, standing_foot);
					*/
				for (char i = 0; i < 4; ++i)
					old_foot[i] = moving_foot[0][i];

				dVector4 predicted_diff;
				subtract_4x1(predicted_diff, predicted_zmp, *moving_foot);
				subtract_4x1(foot_diff, *standing_foot, *moving_foot);

				dReal angle = angleBetween_4x1(foot_diff, predicted_diff);

				if (angle < 0.5)
				{
					angle = 0.5;
				}

				dVector4 axis;
				cross_4x1(axis, foot_diff, predicted_diff);

				dMatrix4 R;
				set_identity_4x4(R);
				dRFromAxisAndAngle(R, axis[0], axis[1], axis[2], angle);

				dVector4 desired_diff;
				multiply_4x4_4x1(desired_diff, R, foot_diff);

				normalize_4x1(desired_diff);
				multiply_4x1_1x1(desired_diff, magnitude_4x1(predicted_diff), desired_diff);
				add_4x1(desired_foot_pos, desired_diff, *moving_foot);
			}
		}

		else if (force[2] > 0.1)
		{
			zmp[0] = position[0];
			zmp[1] = position[1];
			zmp[2] = 0;
			zmp[3] = 1;

			
			//handle jumping/flying case
		}

		else
		{
			zmp[0] = position[0];
			zmp[1] = position[1];
			zmp[2] = 0;
			zmp[3] = 1;
		}
	}

	dBodyID getBody() const
	{
		return body;
	}

	void setRestRotation(const dMatrix4 rotation)
	{
		for (int i = 0; i < 12; ++i)
			rest_rot[i] = rotation[i];
	}

	const dReal* getRestRotation() const
	{
		return rest_rot;
	}

	void setRestPosition(const dVector4 pos)
	{
		for (int i = 0; i < 3; ++i)
			rest_pos[i] = pos[i];
		rest_pos[3] = 1;
	}

	const dReal* getRestPosition() const
	{
		return rest_pos;
	}

	void setFoot1Position(dVector4 position)
	{
		for (int i = 0; i < 2; ++i)
			foot1[i] = position[i] + dBodyGetPosition(body)[i];
	}

	const dReal* getFoot1Position() const
	{
		return foot1;
	}

	void setFoot2Position(dVector4 position)
	{
		for (int i = 0; i < 2; ++i)
			foot2[i] = position[i] + dBodyGetPosition(body)[i];;
	}
	const dReal* getFoot2Position() const
	{
		return foot2;
	}

	const dReal* getZeroMomentPoint() const
	{
		return zmp;
	}

	const dReal* getZeroMomentPointPrediction() const
	{
		return predicted_zmp;
	}
};
#endif