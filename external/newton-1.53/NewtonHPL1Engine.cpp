/* HPL1Engine additions to Julio Jerez's Newton 1.53 compatibility layer.
*
* Newton 1.53 functions that HPL1Engine calls but that the layer declares
* without defining (their old implementations only exist inside #if 0).
* Written for this fork only; never sent upstream.
*
* Rule for every function here: reproduce what real Newton 1.53 returned or
* did, as closely as Newton 4 allows, because the game's tuning (break
* forces, sleep thresholds, character push-out) was made against 1.53.
* Where 1.53's exact code is unavailable, the Newton 2.36 source (the
* closest open relative, used by ScummVM's HPL1 port) is the reference.
*
* Same zlib licence as the rest of the layer (see LICENSE).
*/

#include "newtonStdafx.h"
#include "Newton.h"
#include "NewtonBody.h"
#include "NewtonUtils.h"
#include "newtonWorld.h"

//-----------------------------------------------------------------------------
// World
//-----------------------------------------------------------------------------

// 1.53: model 0 = exact solver (the default, and what Penumbra uses at its
// default "High" physics accuracy), 1 = adaptive, n > 1 = n linear passes.
// Newton 4 has no exact solver, only an iteration count, so the exact and
// adaptive modes map to fixed iteration counts. These two numbers are the
// knobs to tune during in-game feel testing.
#define HPL1_SOLVER_ITERATIONS_EXACT		16
#define HPL1_SOLVER_ITERATIONS_ADAPTIVE		8

void NewtonSetSolverModel(const NewtonWorld* newtonWorld, int model)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	ndInt32 iterations = HPL1_SOLVER_ITERATIONS_EXACT;
	if (model == 1)
	{
		iterations = HPL1_SOLVER_ITERATIONS_ADAPTIVE;
	}
	else if (model > 1)
	{
		iterations = model;
	}
	world->SetSolverIterations(iterations);
}

// 1.53: 0 = exact Coulomb friction, 1 = adaptive. Newton 4 has a single
// friction model, so there is nothing to switch.
void NewtonSetFrictionModel(const NewtonWorld*, int)
{
	TRACE_FUNCTION(__FUNCTION__);
}

//-----------------------------------------------------------------------------
// Bodies
//-----------------------------------------------------------------------------

// 1.53 stored speed^2 and omega^2 limits per body and ignored framesCount
// (its own code had the frame-count logic commented out). See
// ndNewtonBodyDynamic in NewtonBody.h for how they are applied.
void NewtonBodySetFreezeTreshold(const NewtonBody* bodyPtr, dFloat freezeSpeedMag2, dFloat freezeOmegaMag2, int)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBody* const body = PointerCast<ndBody, NewtonBody>(bodyPtr);
	ndNewtonBodyDynamic* const dynBody = dynamic_cast<ndNewtonBodyDynamic*>(body);
	if (dynBody)
	{
		dynBody->m_freezeSpeed2 = ndAbs(freezeSpeedMag2);
		dynBody->m_freezeOmega2 = ndAbs(freezeOmegaMag2);
	}
}

//-----------------------------------------------------------------------------
// Materials and contacts
//-----------------------------------------------------------------------------

// Newton 4 has no per-material continuous collision switch. (HPL1 only
// enables it for the material pairs it configures; the bodies' own
// continuous-collision setting is handled separately.)
void NewtonMaterialSetContinuousCollisionMode(const NewtonWorld*, int, int, int)
{
	TRACE_FUNCTION(__FUNCTION__);
}

// Same as 1.53: the contact's solved normal force plus its two friction
// forces, as one world-space vector. Only valid inside a contact callback,
// where the material handle is the contact point.
void NewtonMaterialGetContactForce(const NewtonMaterial* materialHandle, dFloat* forcePtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	const ndContactMaterial* const material = PointerCast<ndContactMaterial, NewtonMaterial>(materialHandle);
	const ndVector force(
		material->m_normal.Scale(material->m_normal_Force.m_force) +
		material->m_dir0.Scale(material->m_dir0_Force.m_force) +
		material->m_dir1.Scale(material->m_dir1_Force.m_force));
	forcePtr[0] = force.m_x;
	forcePtr[1] = force.m_y;
	forcePtr[2] = force.m_z;
}

//-----------------------------------------------------------------------------
// Collision queries
//-----------------------------------------------------------------------------

// 1.53: contact points between two collision shapes placed at the given
// matrices, with no bodies involved. HPL1's CheckShapeCollision is built on
// this (character controller, ladders, damage/force/liquid/stick areas,
// weapon hits), and pushes shape A out of B by normal * penetration, so:
// normals point from B towards A, and penetration is positive when the
// shapes overlap. Returns the number of points written (at most maxSize).
int NewtonCollisionCollide(const NewtonWorld* newtonWorld, int maxSize,
	const NewtonCollision* collisionA, const dFloat* matrixA,
	const NewtonCollision* collisionB, const dFloat* matrixB,
	dFloat* contacts, dFloat* normals, dFloat* penetration)
{
	TRACE_FUNCTION(__FUNCTION__);
	if (maxSize <= 0)
	{
		return 0;
	}

	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	const ndShapeInstance* const instanceA = PointerCast<ndShapeInstance, NewtonCollision>(collisionA);
	const ndShapeInstance* const instanceB = PointerCast<ndShapeInstance, NewtonCollision>(collisionB);
	const ndMatrix mA(GetMatrix(matrixA));
	const ndMatrix mB(GetMatrix(matrixB));

	// Same pattern Newton 4 itself uses for shape queries (ndScene::Collide).
	ndFixSizeArray<ndContactPoint, 16> points;
	ndContactNotify notification(world->GetScene());
	ndContactSolver solver(*instanceA, *instanceB);
	solver.CalculateContacts(instanceA, mA, ndVector::m_zero, instanceB, mB, ndVector::m_zero, points, &notification);

	const ndInt32 count = ndMin(ndInt32(points.GetCount()), ndInt32(maxSize));
	for (ndInt32 i = 0; i < count; ++i)
	{
		const ndContactPoint& point = points[i];
		contacts[i * 3 + 0] = point.m_point.m_x;
		contacts[i * 3 + 1] = point.m_point.m_y;
		contacts[i * 3 + 2] = point.m_point.m_z;
		// Newton 4's normal here already points from B towards A, as 1.53's
		// did (checked: sphere resting into a box gives +Y).
		normals[i * 3 + 0] = point.m_normal.m_x;
		normals[i * 3 + 1] = point.m_normal.m_y;
		normals[i * 3 + 2] = point.m_normal.m_z;
		penetration[i] = point.m_penetration;
	}
	return count;
}

//-----------------------------------------------------------------------------
// Joints
//-----------------------------------------------------------------------------

// 1.53's GetJointForce summed the solver forces of the joint's base rows
// along their axes. Those rows are all linear (point) rows in 1.53, so the
// sum is the net linear reaction force on the child body; the extra row a
// user callback adds (limits, friction, motors) is not included. HPL1 uses
// the length of this vector against each joint's break force.
//
// Newton 4 records the net force each joint applies to body0 after every
// solve. Its base rows are partly angular, but angular rows carry no linear
// force, so for the ball and the hinge (whose extra rows are angular) that
// net force is exactly 1.53's value. For the slider and the corkscrew the
// limit/motor row is linear along the pin, and 1.53's base rows were all
// perpendicular to the pin, so the pin component is removed.
static void HPL1JointForce(const NewtonJoint* const jointPtr, dFloat* const force, bool removePinComponent)
{
	const ndJointBilateralConstraint* const joint = PointerCast<ndJointBilateralConstraint, NewtonJoint>(jointPtr);
	ndVector f(joint->GetForceBody0() & ndVector::m_triplexMask);
	if (removePinComponent)
	{
		const ndVector pin(joint->CalculateGlobalMatrix0().m_front);
		f = f - pin.Scale(f.DotProduct(pin).GetScalar());
	}
	force[0] = f.m_x;
	force[1] = f.m_y;
	force[2] = f.m_z;
}

void NewtonBallGetJointForce(const NewtonJoint* ball, dFloat* force)
{
	TRACE_FUNCTION(__FUNCTION__);
	HPL1JointForce(ball, force, false);
}

void NewtonHingeGetJointForce(const NewtonJoint* hinge, dFloat* force)
{
	TRACE_FUNCTION(__FUNCTION__);
	HPL1JointForce(hinge, force, false);
}

void NewtonSliderGetJointForce(const NewtonJoint* slider, dFloat* force)
{
	TRACE_FUNCTION(__FUNCTION__);
	HPL1JointForce(slider, force, true);
}

void NewtonCorkscrewGetJointForce(const NewtonJoint* corkscrew, dFloat* force)
{
	TRACE_FUNCTION(__FUNCTION__);
	HPL1JointForce(corkscrew, force, true);
}

// As in 2.36's dgBallConstraint: the relative angular velocity of the two
// bodies, expressed in the joint's child frame.
void NewtonBallGetJointOmega(const NewtonJoint* ball, dFloat* omega)
{
	TRACE_FUNCTION(__FUNCTION__);
	const ndJointBilateralConstraint* const joint = PointerCast<ndJointBilateralConstraint, NewtonJoint>(ball);
	const ndMatrix matrix0(joint->CalculateGlobalMatrix0());
	const ndVector relOmega(joint->GetBody0()->GetOmega() - joint->GetBody1()->GetOmega());
	omega[0] = relOmega.DotProduct(matrix0.m_front).GetScalar();
	omega[1] = relOmega.DotProduct(matrix0.m_up).GetScalar();
	omega[2] = relOmega.DotProduct(matrix0.m_right).GetScalar();
}

// As in 2.36's dgBallConstraint: minus the pitch/yaw/roll angles of the
// parent frame relative to the child frame.
void NewtonBallGetJointAngle(const NewtonJoint* ball, dFloat* angle)
{
	TRACE_FUNCTION(__FUNCTION__);
	const ndJointBilateralConstraint* const joint = PointerCast<ndJointBilateralConstraint, NewtonJoint>(ball);
	ndMatrix matrix0;
	ndMatrix matrix1;
	joint->CalculateGlobalMatrix(matrix0, matrix1);
	const ndMatrix relMatrix(matrix1 * matrix0.OrthoInverse());
	ndVector euler1;
	const ndVector euler0(relMatrix.CalcPitchYawRoll(euler1));
	angle[0] = -euler0.m_x;
	angle[1] = -euler0.m_y;
	angle[2] = -euler0.m_z;
}
