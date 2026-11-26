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

// HPL1Engine: rigid body with 1.53-style per-body freeze thresholds.
// Newton 4 only has world-wide sleep thresholds, but HPL1 sets them per body
// through NewtonBodySetFreezeTreshold (default 0.01 for both speed^2 and
// omega^2, which is much looser than Newton 4's 0.032^2). Bodies that never
// get thresholds keep Newton 4's world values. The class name reported for
// reflection stays "ndBodyDynamic".
class ndNewtonBodyDynamic : public ndBodyDynamic
{
	public:
	ndNewtonBodyDynamic()
		:ndBodyDynamic()
		,m_freezeSpeed2(ndFloat32(-1.0f))
		,m_freezeOmega2(ndFloat32(-1.0f))
		,m_continuousCollision(0)
		,m_freezeFrames(0)
		,m_quietFrames(0)
	{
	}

	virtual void EvaluateSleepState(ndFloat32 freezeSpeed2, ndFloat32 freezeAccel2) override
	{
		if (m_freezeSpeed2 >= ndFloat32(0.0f))
		{
			// Newton 4 tests speed^2 and omega^2 against one value. HPL1
			// normally passes the same value for both; if they differ, use
			// the stricter one.
			freezeSpeed2 = ndMin(m_freezeSpeed2, m_freezeOmega2);
		}

		// HPL1Engine: in 1.53 (as its documentation of framesCount says) a
		// body was put to rest only after staying below its thresholds for
		// framesCount steps in a row, and NewtonWorldUnfreezeBody started
		// that count again. Newton 4 rests a body (and zeroes its velocity)
		// on the first quiet step. HPL1's wave gravity (the boat in level 1)
		// wakes every body each frame so that slow swings keep going;
		// without the count a hanging lamp stopped dead whenever it slowed
		// down. A body already at rest is left alone, and the count starts
		// again from 0 once a body is let rest, so it applies after any wake.
		const bool resting = m_equilibrium != 0;
		ndBodyDynamic::EvaluateSleepState(freezeSpeed2, freezeAccel2);
		if (resting || (m_freezeFrames <= 0))
		{
			return;
		}
		if (!m_isJointFence0)
		{
			m_quietFrames = 0;
		}
		else if (m_quietFrames < m_freezeFrames)
		{
			m_quietFrames++;
			m_isJointFence0 = 0;
			m_equilibrium = 0;
		}
		else
		{
			m_quietFrames = 0;
		}
	}

	ndFloat32 m_freezeSpeed2;
	ndFloat32 m_freezeOmega2;

	// 1.53's per-body continuous collision flag. Newton 4 has no per-body
	// switch, so it is only stored and reported back (HPL1 saves it in its
	// save games and reads it back on load).
	int m_continuousCollision;

	// Steps a body must stay below its thresholds before it may rest
	// (framesCount of NewtonBodySetFreezeTreshold), and how many it has.
	int m_freezeFrames;
	int m_quietFrames;
};

#endif