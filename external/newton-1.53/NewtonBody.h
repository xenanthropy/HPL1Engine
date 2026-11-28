/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef D_NEWTON_BOBY_H_
#define D_NEWTON_BOBY_H_

#include "newtonStdafx.h"
#include "Newton.h"
#include "NewtonUtils.h"
#include "newtonWorld.h"

struct NewtonBody;
struct NewtonJoint;
struct NewtonWorld;

class ndNewtonBodyNotify : public ndModelBodyNotify
{
	public:
	D_CLASS_REFLECTION(ndNewtonBodyNotify, ndModelBodyNotify)

	typedef void (*NewtonBodyDestructor) (const NewtonBody* const body);
	typedef void (*NewtonApplyForceAndTorque) (const NewtonBody* const body);
	typedef void (*NewtonBodyActivationState) (const NewtonBody* body, unsigned state);
	typedef void (*NewtonSetTransform) (const NewtonBody* const body, const ndFloat32* const matrix);

	ndNewtonBodyNotify(ndNewtonWorld* const world, NewtonBody* const owner);
	ndNewtonBodyNotify(const ndNewtonBodyNotify& notify);
	virtual ~ndNewtonBodyNotify() override;

	ndBodyNotify* Clone() const override;
	virtual void OnBodyAddedToWorld();
	virtual void OnBodyRemovedFromWorld();
	virtual void OnTransform(ndFloat32 timestep, const ndMatrix& matrix) override;

	virtual void OnApplyExternalForce(ndInt32 threadIndex, ndFloat32 timestep) override;

	ndWeakPtr<void> m_userData;
	ndWeakPtr<NewtonBody> m_owner;
	ndWeakPtr<ndNewtonWorld> m_world;
	ndInt32 m_materialGoupId;
	ndFloat32 m_capSpeed;
	ndFloat32 m_capOmega;

	bool m_bodyIsInWorld;
	// HPL1Engine: for ndNewtonWorld::GuardBodies.
	ndMatrix m_lastGoodMatrix;
	bool m_hasLastGoodMatrix;
	bool m_nonFiniteReported;
	bool m_collideWithConnectedBodies;

	NewtonBodyDestructor m_onDestroy;
	NewtonSetTransform m_applyTransform;
	NewtonApplyForceAndTorque m_onForceAndTorque;
};

// HPL1Engine: rigid body that sleeps the way Newton 1.53 did. Measured
// against the original 1.53 library (32-bit, test only):
//
// - Each update, after solving, a body is "quiet" when its velocity at the
//   start of the step and its solved acceleration are all below limits taken
//   from NewtonBodySetFreezeTreshold (1.53's default 0.01 when never set):
//     speed^2 < 0.022 * max(freezeSpeed2, 0.1)   accel^2 < 2.64 * max(freezeSpeed2, 0.01)
//     omega^2 < 0.022 * max(freezeOmega2, 0.1)   alpha^2 < 2.64 * max(freezeOmega2, 0.01)
//   framesCount is not used.
// - When a body's whole island is quiet it rests without taking the step:
//   its position and its velocity stay as they were before the step. Newton
//   4 zeroes the velocity and keeps the step.
// - NewtonWorldUnfreezeBody only wakes a body; a quiet body rests again in
//   the same update without moving. A body takes at least one full step after
//   it is created or after NewtonBodySetVelocity/SetOmega.
//
// The class name reported for reflection stays "ndBodyDynamic".
class ndNewtonBodyDynamic : public ndBodyDynamic
{
	public:
	ndNewtonBodyDynamic()
		:ndBodyDynamic()
		,m_freezeSpeed2(ndFloat32(0.01f))
		,m_freezeOmega2(ndFloat32(0.01f))
		,m_continuousCollision(0)
		,m_mustStep(1)
		,m_stepVeloc(ndVector::m_zero)
		,m_stepOmega(ndVector::m_zero)
		,m_stepCentre(ndVector::m_zero)
		,m_stepRotation()
		,m_stepMatrix(ndGetIdentityMatrix())
		,m_madeStatic(false)
	{
	}

	virtual void IntegrateSolverVelocity(ndFloat32 timestep) override
	{
		// m_accel/m_alpha now hold this step's solved accelerations and
		// m_veloc/m_omega the new velocities: keep what the body had before.
		m_stepVeloc = m_veloc - m_accel.Scale(timestep);
		m_stepOmega = m_omega - m_alpha.Scale(timestep);
		m_stepCentre = m_globalCentreOfMass;
		m_stepRotation = m_rotation;
		m_stepMatrix = m_matrix;
		ndBodyDynamic::IntegrateSolverVelocity(timestep);
	}

	virtual void EvaluateSleepState(ndFloat32, ndFloat32) override
	{
		m_isJointFence0 = 1;
		if (m_isStatic)
		{
			m_equilibrium = 1;
			return;
		}
		const ndFloat32 speedLimit2 = ndFloat32(0.022f) * ndMax(m_freezeSpeed2, ndFloat32(0.1f));
		const ndFloat32 omegaLimit2 = ndFloat32(0.022f) * ndMax(m_freezeOmega2, ndFloat32(0.1f));
		const ndFloat32 accelLimit2 = ndFloat32(2.64f) * ndMax(m_freezeSpeed2, ndFloat32(0.01f));
		const ndFloat32 alphaLimit2 = ndFloat32(2.64f) * ndMax(m_freezeOmega2, ndFloat32(0.01f));
		const bool quiet = m_autoSleep && !m_mustStep &&
			(m_stepVeloc.DotProduct(m_stepVeloc).GetScalar() < speedLimit2) &&
			(m_stepOmega.DotProduct(m_stepOmega).GetScalar() < omegaLimit2) &&
			(m_accel.DotProduct(m_accel).GetScalar() < accelLimit2) &&
			(m_alpha.DotProduct(m_alpha).GetScalar() < alphaLimit2);
		m_mustStep = 0;
		m_isJointFence0 = quiet ? 1 : 0;
		if (quiet && !m_isConstrained)
		{
			// Nothing touches it, so its island is just itself.
			m_equilibrium = 1;
			OnIslandRest();
		}
	}

	virtual void OnIslandRest() override
	{
		// Undo this step: back to the position and velocity it had.
		m_veloc = m_stepVeloc;
		m_omega = m_stepOmega;
		m_globalCentreOfMass = m_stepCentre;
		m_rotation = m_stepRotation;
		m_matrix = m_stepMatrix;
	}

	// NewtonBodySetFreezeTreshold's limits (1.53 defaults until set).
	ndFloat32 m_freezeSpeed2;
	ndFloat32 m_freezeOmega2;

	// 1.53's per-body continuous collision flag. Newton 4 has no per-body
	// switch, so it is only stored and reported back (HPL1 saves it in its
	// save games and reads it back on load).
	int m_continuousCollision;

	// Set at creation and by NewtonBodySetVelocity/SetOmega: take the next
	// step even if quiet.
	int m_mustStep;

	// The body's state at the start of the current step.
	ndVector m_stepVeloc;
	ndVector m_stepOmega;
	ndVector m_stepCentre;
	ndQuaternion m_stepRotation;
	ndMatrix m_stepMatrix;

	// HPL1Engine: given mass 0 after it was created (a stick area holding
	// it), see NewtonBodySetFullMassMatrix.
	bool m_madeStatic;
};

#endif