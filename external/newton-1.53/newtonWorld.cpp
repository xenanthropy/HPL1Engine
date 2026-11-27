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

#include "newtonStdafx.h"
#include <cmath>
#include <cstdio>
#include "Newton.h"
#include "NewtonBody.h"
#include "NewtonUtils.h"
#include "newtonWorld.h"
#include "NewtonVehicle.h"
#include "NewtonMaterialGraph.h"

// HPL1Engine: continuous collision limits (see ndNewtonWorld::Update).
#define D_HPL_CONTINUOUS_MAX_PIECES		8
#define D_HPL_CONTINUOUS_MIN_STEP		ndFloat32 (1.0e-4f)
#define D_HPL_CONTINUOUS_MIN_ADVANCE	ndFloat32 (1.0e-3f)

ndSpinLock ndNewtonWorld::m_globalCriticalSection;

class ndNewtonContact : public ndContact
{
public:
	D_CLASS_REFLECTION(ndNewtonContact, ndContact)

		ndNewtonContact(ndNewtonWorld* const world)
		:ndContact()
		, m_world(world)
	{
	}

	ndNewtonContact(const ndNewtonContact& src)
		:ndContact(src)
		, m_world(src.m_world)
	{
	}

	ndContact* Clone() const override
	{
		return new ndNewtonContact(*this);
	}

	void JacobianDerivative(ndConstraintDescritor& desc) override
	{
		ndBodyKinematic* const kinBody0 = GetBody0();
		ndBodyKinematic* const kinBody1 = GetBody1();
		ndNewtonBodyNotify* const bodyNotify0 = PointerCast<ndNewtonBodyNotify, ndBodyNotify>(*kinBody0->GetNotifyCallback());
		ndNewtonBodyNotify* const bodyNotify1 = PointerCast<ndNewtonBodyNotify, ndBodyNotify>(*kinBody1->GetNotifyCallback());
		if (bodyNotify0->m_collideWithConnectedBodies || bodyNotify1->m_collideWithConnectedBodies)
		{
			ndAssert(0);
		}

		//const ndNewtonBodyNotify* const notify0 = PointerCast<ndNewtonBodyNotify, ndBodyNotify>(*kinBody0->GetNotifyCallback());
		//const ndNewtonBodyNotify* const notify1 = PointerCast<ndNewtonBodyNotify, ndBodyNotify>(*kinBody1->GetNotifyCallback());
		//const ndNewtonMaterial* const material = PointerCast<ndNewtonMaterial, ndMaterial>(m_world->GetMaterial(notify0->m_materialGoupId, notify1->m_materialGoupId));
		//ndAssert(material);
		//if (material->m_onContactGeneration)
		//{
		//	//typedef int (*NewtonOnContactGeneration) (
		//	// const NewtonMaterial* const material, 
		//	// const NewtonBody* const body0, const NewtonCollision* const collision0, 
		//	// const NewtonBody* const body1, const NewtonCollision* const collision1, 
		//	// NewtonUserContactPoint* const contactBuffer, int maxCount, int threadIndex);
		//
		//	NewtonUserContactPoint contactBuffer[32];
		//	const NewtonMaterial* const self = PointerCast<NewtonMaterial, ndNewtonMaterial>(material);
		//	const NewtonBody* const body0 = PointerCast<NewtonBody, ndBodyKinematic>(kinBody0);
		//	const NewtonBody* const body1 = PointerCast<NewtonBody, ndBodyKinematic>(kinBody1);
		//	const NewtonCollision* const instance0 = PointerCast<NewtonCollision, ndShapeInstance>(&kinBody0->GetCollisionShape());
		//	const NewtonCollision* const instance1 = PointerCast<NewtonCollision, ndShapeInstance>(&kinBody1->GetCollisionShape());
		//
		//	ndContactPointList& contacPointsList = m_contacPointsList;
		//	ndInt32 count = material->m_onContactGeneration(self, body0, instance0, body1, instance1, contactBuffer, contacPointsList.GetCount(), ndInt32(contacPointsList.GetCount()));
		//	while (contacPointsList.GetCount() > count)
		//	{
		//		contacPointsList.Remove(contacPointsList.GetLast());
		//	}
		//}
		//else
		//{
		//	//ndAssert(0);
		//}
		ndContact::JacobianDerivative(desc);
	}

	ndWeakPtr<ndNewtonWorld> m_world;
};

class ndNewtonContactCallback : public ndContactCallback
{
	public:
	ndNewtonContactCallback(ndScene* const scene)
		:ndContactCallback(scene)
	{
		m_defaultConstructor = ndSharedPtr<ndContact>(new ndNewtonContact(PointerCast<ndNewtonWorld, ndWorld>(scene->GetWorld())));
	}

	// HPL1Engine: pick the contact's material from the two bodies' 1.53
	// material groups (NewtonBodySetMaterialGroupID). The core looks it up
	// by the shapes' user ids instead, which the layer never sets, so every
	// contact used the default material: no per-material friction or
	// elasticity, and none of HPL's contact callbacks (impact sounds,
	// breaking, damage).
	ndMaterial* GetMaterial(const ndContact* const contact, const ndShapeInstance&, const ndShapeInstance&) const override
	{
		return ndContactCallback::GetMaterial(ndUnsigned32(MaterialGroup(contact->GetBody0())), ndUnsigned32(MaterialGroup(contact->GetBody1())));
	}

	static ndInt32 MaterialGroup(ndBodyKinematic* const body)
	{
		const ndBodyNotify* const notify = *body->GetNotifyCallback();
		const ndNewtonBodyNotify* const newtonNotify = notify ? dynamic_cast<const ndNewtonBodyNotify*>(notify) : nullptr;
		return newtonNotify ? newtonNotify->m_materialGoupId : 0;
	}
};

ndNewtonWorld::ndNewtonWorld()
	:ndWorld()
	,m_userData(nullptr)
	,m_bodyMaterialGroup(1)
	,m_onPostUpdate(nullptr)
	,m_onJointSerialize(nullptr)
	,m_onJointDeserialize(nullptr)
	,m_onBodySerialize(nullptr)
	,m_onBodyDeserialize(nullptr)
	,m_onCreateContact(nullptr)
	,m_onDestroyContact(nullptr)
{
	// HPL1Engine: one step per NewtonUpdate, as in 1.53. With 3 substeps
	// every contact and joint callback also ran 3 times per update.
	SetSubSteps(1);
	// HPL1Engine: single-threaded, on the caller's thread (see
	// D_USE_THREAD_EMULATION in CMakeLists.txt). Was: the default.
	SetThreadCount(1);
	// HPL1Engine: the standard solver. The SIMD8 one mixes up the rows of
	// joints and contacts with different row counts in the same batch (seen:
	// a frictionless contact made the next box's contact frictionless too),
	// and with one thread and scalar maths it gains nothing.
	SelectSolver(ndStandardSolver);
	SetContactCallback();
	SetWorldSize(ndVector(-100.0f, -100.0f, -100.0f, 1.0f), ndVector(100.0f, 100.0f, 100.0f, 1.0f));
}

void ndNewtonWorld::SetWorldSize(const ndVector& minBox, const ndVector& maxBox)
{
	m_worldMin = minBox.GetMin(maxBox);
	m_worldMax = minBox.GetMax(maxBox);
}

static bool IsFiniteVector(const ndVector& v)
{
	return std::isfinite(v.m_x) && std::isfinite(v.m_y) && std::isfinite(v.m_z);
}

void ndNewtonWorld::GuardBodies()
{
	const ndBodyListView& bodies = GetBodyList();
	for (ndBodyListView::ndNode* node = bodies.GetFirst(); node; node = node->GetNext())
	{
		ndBodyKinematic* const body = node->GetInfo()->GetAsBodyKinematic();
		if (!body || (body->GetInvMass() <= ndFloat32(0.0f)))
		{
			continue;
		}
		ndNewtonBodyNotify* const notify = dynamic_cast<ndNewtonBodyNotify*>(*body->GetNotifyCallback());
		if (!notify)
		{
			continue;
		}

		const ndMatrix& matrix = body->GetMatrix();
		const bool finite = IsFiniteVector(matrix.m_front) && IsFiniteVector(matrix.m_up) &&
			IsFiniteVector(matrix.m_right) && IsFiniteVector(matrix.m_posit) &&
			IsFiniteVector(body->GetVelocity()) && IsFiniteVector(body->GetOmega());
		if (!finite)
		{
			// NaN/inf would spread to every body and joint touching it and on
			// into the game. Put the body back where it last was valid and stop it.
			if (!notify->m_nonFiniteReported)
			{
				fprintf(stderr, "Newton: a body's position or velocity became NaN/inf; restoring its last valid position and stopping it.\n");
				notify->m_nonFiniteReported = true;
			}
			const ndMatrix restore(notify->m_hasLastGoodMatrix ? notify->m_lastGoodMatrix : ndGetIdentityMatrix());
			body->SetMatrix(restore);
			body->SetVelocity(ndVector::m_zero);
			body->SetOmega(ndVector::m_zero);
			body->SetSleepState(true);
			notify->OnTransform(0.0f, restore);
			continue;
		}
		notify->m_lastGoodMatrix = matrix;
		notify->m_hasLastGoodMatrix = true;

		ndVector p0;
		ndVector p1;
		body->GetAABB(p0, p1);
		const bool inside =
			(p0.m_x >= m_worldMin.m_x) && (p0.m_y >= m_worldMin.m_y) && (p0.m_z >= m_worldMin.m_z) &&
			(p1.m_x <= m_worldMax.m_x) && (p1.m_y <= m_worldMax.m_y) && (p1.m_z <= m_worldMax.m_z);
		if (!inside)
		{
			// 1.53: "When a body moves outside the bounding box that defines the
			// world space the body is automatically disabled." Kept disabled
			// for as long as it stays outside.
			body->SetVelocity(ndVector::m_zero);
			body->SetOmega(ndVector::m_zero);
			body->SetSleepState(true);
		}
	}
}

ndNewtonWorld::~ndNewtonWorld()
{
}

void ndNewtonWorld::SetContactCallback()
{
	ndContactCallback* const callback = new ndNewtonContactCallback(GetScene());
	ndSharedPtr<ndApplicationMaterial> defaultMaterial(new ndNewtonMaterial);
	callback->SetDefualtMaterial(defaultMaterial);
	SetContactNotify(ndSharedPtr<ndContactNotify>(callback));
}

void ndNewtonWorld::OnAddBody(ndBody* const body) const
{
	ndWorld::OnAddBody(body);
}

void ndNewtonWorld::OnRemoveBody(ndBody* const body) const
{
	ndWorld::OnRemoveBody(body);
}

void ndNewtonWorld::OnAddJoint(ndJointBilateralConstraint* const joint) const
{
	ndWorld::OnAddJoint(joint);
}

void ndNewtonWorld::OnRemoveJoint(ndJointBilateralConstraint* const joint) const
{
	ndWorld::OnRemoveJoint(joint);
}

//void ndNewtonWorld::ClearMaterials()
//{
//	SetContactCallback();
//}

void ndNewtonWorld::OnInitSkeleton(ndSkeletonContainer* const skeleton) const 
{
	ndWorld::OnInitSkeleton(skeleton);
	skeleton->SetAutoRegualrization(false, ndFloat32(2.0f) * D_DEFAULT_REGULARIZER);
	// HPL1Engine: but keep the fallback that stiffens the diagonal when the
	// skeleton's contact and limit rows make the matrix not positive definite
	// (redundant rows). Without it the solve returned forces of 1e14 and the
	// whole chain went NaN: the mine cable, plugged in by one end and lying on
	// the floor against its ball joint limits (level01_10). 1.53 has no such
	// failure. Only matrices that fail the test are changed.
	skeleton->SetLcpRegularizationFallback(true);
}

ndMaterial* ndNewtonWorld::GetMaterial(int id0, int id1) const
{
	ndContactCallback* const notify = static_cast<ndContactCallback*>(*GetContactNotify());
	if (!notify->HasMaterial(id0, id1))
	{
		if ((id0 < m_bodyMaterialGroup) && (id1 < m_bodyMaterialGroup))
		{
			// HPL1Engine: also give pairs with the default group 0 their own
			// material (HPL's "Default" material is group 0). Only 0/0 uses
			// the shared default material. Was: both ids non-zero.
			if ((id0 != 0) || (id1 != 0))
			{
				ndNewtonMaterial material;
				notify->RegisterMaterial(material, id0, id1);
			}
		}
	}
	return notify->GetMaterial(id0, id1);
}

void ndNewtonWorld::PostUpdate(ndFloat32 timestep)
{
	GuardBodies();
	if (m_onPostUpdate)
	{
		m_onPostUpdate(PointerCast<NewtonWorld, ndNewtonWorld>(this), timestep);
	}
}

// HPL1Engine: as in the 1.53 library, a resting body joined to an awake
// dynamic body wakes at the start of the next update, so it moves with it
// and gets its transform callback. HPL1 rests a body that sits at a joint
// limit (LimitAutoSleep) after every step; Newton 4 then dragged it along
// through the joint but left it marked asleep and never reported its new
// matrix, so the outside hatch's wheel stayed behind on screen while the
// hatch opened.
void ndNewtonWorld::WakeJoinedBodies()
{
	bool changed = true;
	while (changed)
	{
		changed = false;
		const ndJointList& joints = GetJointList();
		for (ndJointList::ndNode* node = joints.GetFirst(); node; node = node->GetNext())
		{
			ndJointBilateralConstraint* const joint = *node->GetInfo();
			ndBodyKinematic* const body0 = joint->GetBody0();
			ndBodyKinematic* const body1 = joint->GetBody1();
			if ((body0->GetInvMass() <= ndFloat32(0.0f)) || (body1->GetInvMass() <= ndFloat32(0.0f)))
			{
				continue;
			}
			const bool asleep0 = body0->GetSleepState();
			const bool asleep1 = body1->GetSleepState();
			if (asleep0 != asleep1)
			{
				(asleep0 ? body0 : body1)->SetSleepState(false);
				changed = true;
			}
		}
	}
}

// HPL1Engine: Newton 1.53's continuous collision. HPL1 turns it on for every
// object body and every material pair. Measured with the original library: a
// can-sized cylinder fired at a level wall bounces off at any speed up to 50
// m/s, where Newton 4 (which has no continuous collision in its world step)
// lets it pass from 5 m/s. 1.53 and 2.36 sweep a fast body along its path for
// the step and solve its island in smaller steps when the sweep finds
// something. Here: each fast body with the switch on is swept with Newton 4's
// shape cast; if it would hit something during the step, the whole step is
// split at the moment of impact (at most 8 pieces per update, as 2.36), so
// the contact, its bounce and HPL's contact callbacks happen as usual. Steps
// where nothing is about to be hit run unchanged.
class ndHplContinuousCast : public ndConvexCastNotify
{
	public:
	ndHplContinuousCast(ndNewtonWorld* const world, ndBodyKinematic* const body)
		:ndConvexCastNotify()
		,m_world(world)
		,m_body(body)
		,m_group(ndNewtonContactCallback::MaterialGroup(body))
	{
	}

	virtual ndUnsigned32 OnRayPrecastAction(const ndBody* const bodyPtr, const ndShapeInstance* const) override
	{
		ndBodyKinematic* const other = const_cast<ndBody*>(bodyPtr)->GetAsBodyKinematic();
		if (!other || (other == m_body) || other->GetAsBodyTriggerVolume())
		{
			return 0;
		}
		for (ndBodyKinematic::ndJointList::ndNode* node = m_body->GetJointList().GetFirst(); node; node = node->GetNext())
		{
			ndJointBilateralConstraint* const joint = node->GetInfo();
			if (((joint->GetBody0() == other) || (joint->GetBody1() == other)) && !joint->IsCollidable())
			{
				return 0;
			}
		}
		const ndMaterial* const material = m_world->GetMaterial(m_group, ndNewtonContactCallback::MaterialGroup(other));
		if (!(material->m_flags & ndContactOptions::m_collisionEnable) ||
			!(material->m_userFlags & D_HPL_MATERIAL_CONTINUOUS_COLLISION))
		{
			return 0;
		}
		return 1;
	}

	virtual ndFloat32 OnRayCastAction(const ndContactPoint&, ndFloat32 param) override
	{
		return param;
	}

	ndNewtonWorld* m_world;
	ndBodyKinematic* m_body;
	ndInt32 m_group;
};

// Sweeps one convex shape by step; returns the fraction of the step at which
// it first hits something it collides with, or 1.
ndFloat32 ndNewtonWorld::CastTimeOfImpact(ndBodyKinematic* const body, const ndShapeInstance& shape, const ndMatrix& origin, const ndVector& step, ndFloat32 travel)
{
	ndHplContinuousCast cast(this, body);
	if (GetScene()->ConvexCast(cast, shape, origin, origin.m_posit + step) && (cast.m_param < ndFloat32(1.0f)))
	{
		// A body already touching (or this close to) what it hits has a
		// contact this step, which handles it.
		if ((cast.m_param * travel) > D_HPL_CONTINUOUS_MIN_ADVANCE)
		{
			return cast.m_param;
		}
	}
	return ndFloat32(1.0f);
}

// Returns the earliest time of impact within timestep of a fast body with
// continuous collision on, or timestep when there is none.
ndFloat32 ndNewtonWorld::ContinuousCollisionTimeOfImpact(ndFloat32 timestep)
{
	ndFloat32 timeOfImpact = timestep;
	const ndBodyListView& bodies = GetBodyList();
	for (ndBodyListView::ndNode* node = bodies.GetFirst(); node; node = node->GetNext())
	{
		ndNewtonBodyDynamic* const body = dynamic_cast<ndNewtonBodyDynamic*>(*node->GetInfo());
		if (!body || !body->m_continuousCollision || (body->GetInvMass() <= ndFloat32(0.0f)) || body->GetSleepState())
		{
			continue;
		}

		// 2.36's test: the step moves the body more than a quarter of its
		// smallest radius (its spin adds up to its max - min radius).
		const ndShapeInstance& shape = body->GetCollisionShape();
		const ndFloat32 minRadius = shape.GetBoxMinRadius();
		const ndVector veloc(body->GetVelocity());
		const ndVector omega(body->GetOmega());
		const ndFloat32 speed = ndSqrt(veloc.DotProduct(veloc).GetScalar());
		const ndFloat32 spin2 = omega.DotProduct(omega).GetScalar() * timestep * timestep;
		const ndFloat32 padding = (shape.GetBoxMaxRadius() - minRadius) * ndMin(ndSqrt(spin2), ndFloat32(1.0f));
		const ndFloat32 travel = speed * timestep;
		if ((travel + padding) <= (ndFloat32(0.25f) * minRadius))
		{
			continue;
		}

		// Newton 4 can't cast a compound shape: cast each of its parts.
		const ndMatrix& matrix = body->GetMatrix();
		const ndVector step(veloc.Scale(timestep));
		ndFloat32 param = ndFloat32(1.0f);
		const ndShapeCompound* const compound = const_cast<ndShape*>(shape.GetShape())->GetAsShapeCompound();
		if (compound)
		{
			const ndMatrix origin(shape.GetLocalMatrix() * matrix);
			ndShapeCompound::ndTreeArray::Iterator it(compound->GetTree());
			for (it.Begin(); it; it++)
			{
				const ndShapeInstance* const part = compound->GetShapeInstance(it.GetNode());
				if (const_cast<ndShape*>(part->GetShape())->GetAsShapeConvex())
				{
					param = ndMin(param, CastTimeOfImpact(body, *part, origin, step, travel));
				}
			}
		}
		else if (const_cast<ndShape*>(shape.GetShape())->GetAsShapeConvex())
		{
			param = CastTimeOfImpact(body, shape, matrix, step, travel);
		}
		timeOfImpact = ndMin(timeOfImpact, param * timestep);
	}
	return timeOfImpact;
}

void ndNewtonWorld::Update(ndFloat32 timestep)
{
	//ndTrace(("%f\n", timestep));
	ndFloat32 remaining = timestep;
	for (ndInt32 piece = 1; ; ++piece)
	{
		WakeJoinedBodies();
		ndFloat32 step = remaining;
		if (piece < D_HPL_CONTINUOUS_MAX_PIECES)
		{
			step = ndMax(ContinuousCollisionTimeOfImpact(remaining), D_HPL_CONTINUOUS_MIN_STEP);
		}
		if (step >= remaining)
		{
			ndWorld::Update(remaining);
			break;
		}
		ndWorld::Update(step);
		Sync();
		remaining -= step;
	}
}


/*!
  Create an instance of the Newton world.

  @return Pointer to new Newton world.

  This function must be called before any of the other API functions.

  See also: ::NewtonDestroy, ::NewtonDestroyAllBodies
*/
NewtonWorld* NewtonCreate(NewtonAllocMemory malloc, NewtonFreeMemory mfree)
{
	TRACE_FUNCTION(__FUNCTION__);

	NewtonSetMemorySystem(malloc, mfree);
	ndNewtonWorld* const world = new ndNewtonWorld();
	return PointerCast<NewtonWorld, ndNewtonWorld>(world);
}

/*!
  Destroy an instance of the Newton world.

  @param *newtonWorld Pointer to the Newton world.
  @return Nothing.

  This function will destroy the entire Newton world.

  See also: ::NewtonCreate, ::NewtonDestroyAllBodies
*/
void NewtonDestroy(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	delete world;
}

/*!
  Return the current library version number.

  @return version number as an integer, eg 314.

  The version number is a three-digit integer.

  First digit:  major version (interface changes among other things)
  Second digit: major patch number (new features, and bug fixes)
  Third Digit:  minor bug fixed patch.
*/
int NewtonWorldGetVersion(const NewtonWorld*)
{
	TRACE_FUNCTION(__FUNCTION__);
	return NEWTON_MAJOR_VERSION * 100 + NEWTON_MINOR_VERSION;
}

// Name: NewtonUpdate 
// Advance the simulation by an amount of time.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world
// *dFloat* timestep - time step in seconds
// 
// Return: Nothing
//
// Remarks: This function will advance the simulation by the amount of time specified by *timestep*. The Newton Engine does 
// not perform sub-steps, and does not need tuning parameters. It is the responsibility of the application to 
// ensure that *timestep* is small enough to guarantee physics stability. 
void NewtonUpdate(const NewtonWorld* newtonWorld, dFloat timestep)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);

	world->Update(timestep);
	world->Sync();
}


// Name: NewtonWorldFreezeBody 
// Remove the body from the active simulation list.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world
// *const NewtonBody* *body - is the pointer to the body to be frozen
// 
// Return: Nothing.
//
// Remarks: When a body is created it is automatically placed in the active simulation list. As an optimization 
// for large scenes, you may use this function to put background bodies in an inactive equilibrium state.
//
// Remarks: This function tells Newton that this body does not currently need to be simulated.
// However, if the body is part of a larger configuration it may be affected indirectly by the reaction forces
// of objects that it is connected to.
//
// See also: NewtonWorldUnfreezeBody, NewtonBodySetAutoFreeze, NewtonBodyGetAutoFreeze
void NewtonWorldFreezeBody(const NewtonWorld* newtonWorld, const NewtonBody* bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	// HPL1Engine: implemented (was an empty stub). Put the body at rest, the
	// counterpart of NewtonWorldUnfreezeBody below. As in the 1.53 library,
	// it keeps its velocity and carries on with it when woken. HPL1 uses this
	// to disable bodies (e.g. doors that settle at a joint limit).
	ndBodyKinematic* const body = PointerCast<ndBody, NewtonBody>(bodyPtr)->GetAsBodyKinematic();
	body->SetSleepState(true);
}

// Name: NewtonWorldUnfreezeBody 
// Add the body to the active simulation list.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world
// *const NewtonBody* *body - is the pointer to the body to be activated
// 
// Return: Nothing
//
// Remarks: This function tells Newton to simulate the rigid body *body*.
//
// See also: NewtonWorldFreezeBody, NewtonBodySetAutoFreeze, NewtonBodyGetAutoFreeze
void NewtonWorldUnfreezeBody(const NewtonWorld* newtonWorld, const NewtonBody* bodyPtr)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndBodyKinematic* const body = PointerCast<ndBody, NewtonBody>(bodyPtr)->GetAsBodyKinematic();
	body->SetSleepState(false);
}

// Name: NewtonWorldRayCast 
// Shoot a ray from p0 to p1 and call the application callback with each ray intersection.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the world.
// *const dFloat* *p0 - pointer to an array of at least three floats containing the beginning of the ray in global space.
// *const dFloat* *p1 - pointer to an array of at least three floats containing the end of the ray in global space.
// *NewtonWorldRayFilterCallback* filter - user define function to be called for each body hit during the ray scan.
// *void* *userData - user data to be passed to the filter callback.
// *NewtonWorldRayPrefilterCallback* prefilter - user define function to be called for each body before intesection.
//
// Return: nothing
// 
// Remarks: The ray cast function will call the application with each body intersecting the line segment. 
// By writing the callback filter function in different ways the application can implement different flavors of ray casting. 
// For example an all body ray cast can be easily implemented by having the filter function always returning 1.0, and copying each 
// rigid body into an array of pointers; a closest hit ray cast can be implemented by saving the body with the smaller intersection 
// parameter and returning the parameter t; and a report the first body hit can be implemented by having the filter function returning 
// zero after the first call and saving the pointer to the rigid body.
//
// Remarks: The most common use for the ray cast function is the closest body hit, In this case it is important, for performance reasons, 
// that the filter function returns the intersection parameter. If the filter function returns a value of zero the ray cast will terminate 
// immediately.
//
// Remarks: if prefilter is not NULL, Newton will call the application right before executing the intersections between the ray and the primitive.
// if the function returns zero the Newton will not raycast the primitive. passing a NULL pointer will raycast the. 
// The application can use this implement faster or smarter filters when implementing complex logic, otherwise for normal all ray cast
// this parameter could be NULL.
//
// Remarks: The ray cast function is provided as an utility function, this means that even thought the function is very high performance 
// by function standards, it can not by batched and therefore it can not be an incremental function. For example the cost of calling 1000 
// ray cast is 1000 times the cost of calling one ray cast. This is much different than the collision system where the cost of calculating 
// collision for 1000 pairs in much, much less that the 1000 times the cost of one pair. Therefore this function must be used with care, 
// as excessive use of it can degrade performance.
void NewtonWorldRayCast(const NewtonWorld* newtonWorld, const dFloat* p0, const dFloat* p1, NewtonWorldRayFilterCallback filter, void* userData, NewtonWorldRayPrefilterCallback prefilter)
{
	TRACE_FUNCTION(__FUNCTION__);
	if (filter)
	{
		class WorldRayCast : public ndRayCastClosestHitCallback
		{
			public:
			WorldRayCast(ndNewtonWorld* const world, void* const userData, NewtonWorldRayFilterCallback filter, NewtonWorldRayPrefilterCallback prefilter)
				:ndRayCastClosestHitCallback()
				,m_userData(userData)
				,m_world(world)
				,m_filter(filter)
				,m_prefilter(prefilter)
			{
			}

			ndUnsigned32 OnRayPrecastAction(const ndBody* const body, const ndShapeInstance* const instance) override
			{
				if (m_prefilter)
				{
					return m_prefilter(PointerCast<NewtonBody, ndBody>(body), PointerCast<NewtonCollision, ndShapeInstance>(instance), m_userData);
				}
				return true;
			}

			ndFloat32 OnRayCastAction(const ndContactPoint& contact, ndFloat32 intersetParam) override
			{
				if (m_filter)
				{
					intersetParam = m_filter(PointerCast<NewtonBody, ndBody>(contact.m_body0), &contact.m_normal[0], int (contact.m_shapeId0), m_userData, intersetParam);
				}
				return intersetParam;
			}

			void* m_userData;
			ndNewtonWorld* m_world;
			NewtonWorldRayFilterCallback m_filter;
			NewtonWorldRayPrefilterCallback m_prefilter;
		};

		ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
		WorldRayCast rayCaster(world, userData, filter, prefilter);
		const ndVector pp0(p0[0], p0[1], p0[2], ndFloat32(0.0f));
		const ndVector pp1(p1[0], p1[1], p1[2], ndFloat32(0.0f));
		world->RayCast(rayCaster, pp0, pp1);
	}
}

// Name: NewtonWorldForEachBodyDo 
// Iterate thought every body in the world calling the function callback.
//
// Parameters:
// *const NewtonWorld* *newtonWorld - is the pointer to the Newton world.
// *NewtonBodyIterator* callback - application define callback 
//
// Return: nothing
// 
// Remarks: The application can call this function to iterate thought every body in the world. 
// the application should provide the function *NewtonBodyIterator callback* to be called by Newton for every body in the world 
//
// Remarks: this function affect severally the performance of Newton. The application should call this function only for debugging purpose
// See also: NewtonBodyForEachPolygonDo, NewtonWorldForEachBodyInAABBDo
void NewtonWorldForEachBodyDo(const NewtonWorld* newtonWorld, NewtonBodyIterator callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);

	const ndBodyListView& bodyList = world->GetBodyList();
	for (ndBodyListView::ndNode* node = bodyList.GetFirst()->GetNext(); node; node = node->GetNext())
	{
		ndSharedPtr<ndBody>& body = node->GetInfo();
		callback(PointerCast<NewtonBody, ndBody>(*body));
	}

	class VehicleDebug : public ndConstraintDebugCallback
	{
		public:
		VehicleDebug(NewtonBodyIterator callback, NewtonConvexCastVehicle* const owner)
			:ndConstraintDebugCallback()
			,m_callback(callback)
			,m_owner(owner)
		{
		}

		void DrawPoint(const ndVector& point, const ndVector& color, ndFloat32 thickness) override
		{
			m_owner->m_onDebugDisplay(&point[0], &point[0], &color[0]);
		}

		void DrawLine(const ndVector& p0, const ndVector& p1, const ndVector& color, ndFloat32 thickness) override
		{
			m_owner->m_onDebugDisplay(&p0[0], &p1[0], &color[0]);
		}

		void DrawTriangle(const ndVector& p0, const ndVector& p1, const ndVector& p2, const ndVector& color) override
		{
			ndAssert(0);
		}

		NewtonBodyIterator m_callback;
		ndWeakPtr<NewtonConvexCastVehicle> m_owner;
	};


	const ndModelList& modelList = world->GetModelList();
	for (ndModelList::ndNode* node = modelList.GetFirst(); node; node = node->GetNext())
	{
		NewtonConvexCastVehicle* const vehicle = PointerCast<NewtonConvexCastVehicle, ndMultiBodyVehicle>(node->GetInfo()->GetAsMultiBodyVehicle());
		if (vehicle)
		{
			for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* tireNode = vehicle->GetTireList().GetFirst(); tireNode; tireNode = tireNode->GetNext())
			{
				ndBodyKinematic* const tireBody = tireNode->GetInfo()->GetBody0();
				callback(PointerCast<NewtonBody, ndBody>(tireBody));
			}
			if (vehicle->m_onDebugDisplay)
			{
				VehicleDebug vehDebug(callback, vehicle);
				vehicle->Debug(vehDebug);
			}
		}
	}
}

void NewtonWorldSetUserData(const NewtonWorld* newtonWorld, void* userData)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	world->m_userData = userData;
}

void* NewtonWorldGetUserData(const NewtonWorld* newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	return *world->m_userData;
}

NewtonPostUpdateCallback NewtonGetPostUpdateCallback(const NewtonWorld* const newtonWorld)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	return world->m_onPostUpdate;
}

void NewtonSetPostUpdateCallback(const NewtonWorld* const newtonWorld, NewtonPostUpdateCallback callback)
{
	TRACE_FUNCTION(__FUNCTION__);
	ndNewtonWorld* const world = PointerCast<ndNewtonWorld, NewtonWorld>(newtonWorld);
	world->m_onPostUpdate = callback;
}
