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
		ndBodyDynamic::EvaluateSleepState(freezeSpeed2, freezeAccel2);
	}

	ndFloat32 m_freezeSpeed2;
	ndFloat32 m_freezeOmega2;
};

#endif