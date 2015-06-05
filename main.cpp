#define dDOUBLE 1

//ODE files
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

//assimp
//#include <assimp/cimport.h>
//#include <assimp/scene.h>
//#include <assimp/postprocess.h>

//STD files
#include <cmath>
#include <ctime>
using namespace std;

//helper functions
#include "helpers.h"
//#include "Animation.h"
#include "Skeleton.h"
#include "rigidbodycontrol.h"
#include "HybridControl2.h"

// global variables
dWorldID world;
dSpaceID space;
dBodyID body;
dBodyID ball_body;
dGeomID ball_geom;
dMass ball_mass;

dGeomID geom;
dGeomID ground;

dMass mass;
dVector3 force;

//const aiScene* scene;
//Pose jumpy;
RigidBodyControl body_control;
HybridControl2 hybrid_control2;
Skeleton skelly;

// simulation functions
void start()
{
	world = dWorldCreate();
	space = dSimpleSpaceCreate(NULL);

	geom = dCreateCapsule(NULL, 0.5, 2.0);
	ball_geom = dCreateSphere(NULL, 0.5);
	ground = dCreatePlane(NULL, 0, 0, 1, 0);

	body = dBodyCreate(world);
	ball_body = dBodyCreate(world);

	dGeomSetBody(geom, body);
	dGeomSetBody(ball_geom, ball_body);

	dMassSetCapsuleTotal(&mass, 80, 3, 0.5, 2.0);
	dMassSetSphereTotal(&ball_mass, 10, 0.5);

	dBodySetMass(body, &mass);
	dBodySetMass(ball_body, &ball_mass);

	dBodySetPosition(body, 0, 0, .5);

	dVector4 rest = {0, 0, 2.25, 1};
	dMatrix3 rest_orientation = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};

	body_control = RigidBodyControl(world, body, rest, rest_orientation);

	skelly = Skeleton(world, 82);
	hybrid_control2 = HybridControl2(&skelly, &body_control);

	//dWorldSetGravity(world, 0, 0, -0.981);
	dBodySetGravityMode(body, 1);
}

void step(int pause)
{
	static clock_t ticks = clock();
	static clock_t oldticks = clock();
	static dReal time_step = ticks - oldticks;
	static unsigned frame = 0;

	ticks = clock();
	time_step = dReal(ticks-oldticks)/CLOCKS_PER_SEC;
	while (time_step < 1/60.0)
	{
		ticks = clock();
		time_step = dReal(ticks-oldticks)/CLOCKS_PER_SEC;
	}

	dWorldStep(world, double(ticks-oldticks)/CLOCKS_PER_SEC);
	oldticks = ticks;

	/* Gravity */
	dBodyAddForce(body, 0, 0, -800);
	dBodyAddForce(ball_body, 0, 0, -200);

	/* Collision detection */
	dReal repulsionConstant = 40000;
	dContactGeom contactPoints [10];
	int num_collides = dCollide(ball_geom, geom, 10, contactPoints, sizeof(dContactGeom)); 
	if (num_collides > 0)
	{
		dVector3 dir;
		dReal contactForce = repulsionConstant * contactPoints[0].depth;

		multiply_4x1_1x1(dir, contactForce, contactPoints[0].normal);

		dBodyAddForceAtPos(body, -dir[0], -dir[1], -dir[2],
			contactPoints[0].pos[0], contactPoints[0].pos[1], contactPoints[0].pos[2]);
		dBodyAddForceAtPos(ball_body, dir[0], dir[1], dir[2],
			contactPoints[0].pos[0], contactPoints[0].pos[1], contactPoints[0].pos[2]);
	}
	num_collides = dCollide(ball_geom, ground, 10, contactPoints, sizeof(dContactGeom)); 
	if (num_collides > 0)
	{
		dVector3 dir;
		dReal contactForce = repulsionConstant * contactPoints[0].depth;

		multiply_4x1_1x1(dir, contactForce, contactPoints[0].normal);

		dBodyAddForceAtPos(ball_body, dir[0], dir[1], dir[2],
			contactPoints[0].pos[0], contactPoints[0].pos[1], contactPoints[0].pos[2]);
	}
	dReal sides[3] = {0.5, 0.5, 0.5};

	dBodyAddForce(body, 300, 0, 0);

	/* Drawing Bodies*/
	hybrid_control2.draw();
	dsDrawSphereD(dBodyGetPosition(ball_body), dBodyGetRotation(ball_body), 0.5);

	/* End Drawing Bodies*/

	const dReal* origin = dBodyGetPosition(body);
	dVector3 pointOfImpact;
	dVector3 offset = {0, 0, 1};
	add_4x1(pointOfImpact, origin, offset);
	dVector3 result;
	add_4x1(result, force, pointOfImpact);
	dsDrawLineD(result, pointOfImpact);

	static int frame_count = 0;
	if (frame_count++ > 3)
	{
		force[0] = 0; force[1] = 0; force[2] = 0;
		frame_count = 0;
	}
	else
	{
		dBodyAddForceAtRelPos(body, force[0], force[1], force[2], 0, 0, 1);
	}

	hybrid_control2.act();
}

void command(int command)
{
	const int force_size = 1500;
	const dReal* p = dBodyGetPosition(body);
	switch (command)
	{
	case 'd':

		dBodySetPosition(ball_body, p[0], p[1]-2.5, p[2]+1);
		dBodySetLinearVel(ball_body, 0, 12, 0);		
		/*force[0] = 0;
		force[1] = force_size;
		force[2] = 0;
		force[3] = 0;

		dBodyAddForceAtRelPos(body, force[0], force[1], force[2], 0, 0, 1);
		*/
		break;
	case 'a':
		dBodySetPosition(ball_body, p[0], p[1]+2.5, p[2]+1);
		dBodySetLinearVel(ball_body, 0, -12, 0);	
		/*
		force[0] = 0;
		force[1] = -force_size;
		force[2] = 0;
		force[3] = 0;

		//dVector3 bunga;
		//multiply_4x4_4x1(bunga, dBodyGetRotation(body), force);

		//std::cout << bunga[0] << ' ' << bunga[1] << ' ' << bunga[2] << std::endl << std::endl;
		dBodyAddForceAtRelPos(body, force[0], force[1], force[2], 0, 0, 1);
		//dBodyAddTorque(body, 1, 0, 0);*/
		break;
	case 's':
		dBodySetPosition(ball_body, p[0]-2.5, p[1], p[2]+1);
		dBodySetLinearVel(ball_body, 12, 0, 0);	
		/*force[0] = force_size;
		force[1] = 0;
		force[2] = 0;
		force[3] = 0;

		//dVector3 bunga;
		//multiply_4x4_4x1(bunga, dBodyGetRotation(body), force);

		//std::cout << bunga[0] << ' ' << bunga[1] << ' ' << bunga[2] << std::endl << std::endl;
		dBodyAddForceAtRelPos(body, force[0], force[1], force[2], 0, 0, 1);
		//dBodyAddTorque(body, 1, 0, 0);*/
		break;
	case 'w':
		dBodySetPosition(ball_body, p[0]+2.5, p[1], p[2]+1);
		dBodySetLinearVel(ball_body, -12, 0, 0);	
		/*force[0] = -force_size;
		force[1] = 0;
		force[2] = 0;
		force[3] = 0;

		dBodyAddForceAtRelPos(body, force[0], force[1], force[2], 0, 0, 1);*/
		break;
	default:
		break;
	}
}

void stop()
{
	dBodyDestroy(body);
	skelly.destroy();
	dWorldDestroy(world);
}

int main(int argc, char **argv)
{
	//scene = aiImportFile("C:\\Users\\Faris\\Documents\\Graphics\\BVH files\\05\\05_01.bvh", 0);
	//jumpy.initialize(scene, scene->mRootNode);

    // setup pointers to drawstuff callback functions
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &step;
    fn.command = &command;
    fn.stop = stop;
    fn.path_to_textures = "../../drawstuff/textures";
    
    // create world
    dInitODE();

    // run demo
    dsSimulationLoop (argc, argv, 800, 600, &fn);

    dCloseODE();
	//aiReleaseImport(scene);
    return 0;
}