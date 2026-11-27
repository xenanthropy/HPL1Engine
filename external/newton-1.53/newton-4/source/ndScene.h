/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef ND_SCENE_H_
#define ND_SCENE_H_

#include "ndCollisionStdafx.h"
#include "ndBvhNode.h"
#include "ndBodyListView.h"
#include "ndContactArray.h"
#include "ndPolygonMeshDesc.h"

#define D_SCENE_MAX_STACK_DEPTH		256

// HPL1Engine: the stack for the best-first walk of the broadphase tree in
// RayCast and ConvexCast. Bodies added between updates are inserted into the
// tree one by one and it is only rebalanced in the next update, so while a
// map loads (HPL1 builds its AI node graph with hundreds of ray casts before
// the first update) the tree can be far deeper than D_SCENE_MAX_STACK_DEPTH.
// The fixed 256-entry arrays then overflowed and corrupted the stack. This
// keeps 256 entries inline, so the usual case does not allocate, and moves to
// the heap only when it needs more.
template<class T>
class ndSceneTraversalStack: public ndClassAlloc
{
	public:
	ndSceneTraversalStack()
		:ndClassAlloc()
		,m_heap()
		,m_data(m_fixed)
		,m_count(0)
		,m_capacity(D_SCENE_MAX_STACK_DEPTH)
	{
	}

	ndInt32 GetCount() const { return m_count; }
	T& operator[] (ndInt32 i) { ndAssert(i >= 0 && i < m_count); return m_data[i]; }
	const T& operator[] (ndInt32 i) const { ndAssert(i >= 0 && i < m_count); return m_data[i]; }
	T Pop() { ndAssert(m_count > 0); m_count--; return m_data[m_count]; }

	void PushBack(const T& element)
	{
		if (m_count == m_capacity)
		{
			const ndInt32 capacity = m_capacity * 2;
			// Resize keeps what the heap array already holds.
			m_heap.Resize(capacity);
			m_heap.SetCount(capacity);
			if (m_data == m_fixed)
			{
				ndMemCpy(&m_heap[0], m_fixed, m_count);
			}
			m_data = &m_heap[0];
			m_capacity = capacity;
		}
		m_data[m_count] = element;
		m_count++;
	}

	private:
	ndSceneTraversalStack(const ndSceneTraversalStack&) = delete;
	ndSceneTraversalStack& operator=(const ndSceneTraversalStack&) = delete;

	T m_fixed[D_SCENE_MAX_STACK_DEPTH];
	ndArray<T> m_heap;
	T* m_data;
	ndInt32 m_count;
	ndInt32 m_capacity;
};

class ndWorld;
class ndScene;
class ndContact;
class ndRayCastNotify;
class ndContactNotify;
class ndConvexCastNotify;
class ndBodiesInAabbNotify;
class ndJointBilateralConstraint;


class ndSceneTreeNotiFy : public ndClassAlloc
{
	public:
	ndSceneTreeNotiFy()
	{
	}

	virtual ~ndSceneTreeNotiFy()
	{
	}

	virtual void OnDebugNode(const ndBvhNode* const node) = 0;
};

D_MSV_NEWTON_CLASS_ALIGN_32
class ndScene : public ndThreadPool
{
	protected:
	class ndContactPairs
	{
		public:
		ndContactPairs(ndUnsigned32 body0, ndUnsigned32 body1)
			:m_body0(ndMin(body0, body1))
			,m_body1(ndMax(body0, body1))
		{
		}

		ndUnsigned32 m_body0;
		ndUnsigned32 m_body1;
	};

	public:
	D_COLLISION_API virtual ~ndScene() override;
	D_COLLISION_API bool ValidateScene();

	D_COLLISION_API virtual bool AddBody(const ndSharedPtr<ndBody>& body);
	D_COLLISION_API virtual bool RemoveBody(const ndSharedPtr<ndBody>& body);

	D_COLLISION_API ndSharedPtr<ndBody> GetBody(ndBody* const body) const;

	D_COLLISION_API virtual void Begin();
	D_COLLISION_API virtual void End();
	D_COLLISION_API virtual void Sync();
	D_COLLISION_API virtual bool IsHighPerformanceCompute() const;
	D_COLLISION_API virtual bool IsValid() const;

	D_COLLISION_API virtual void Cleanup();
	D_COLLISION_API virtual void PrepareCleanup();

	D_COLLISION_API ndSharedPtr<ndContactNotify> GetContactNotify() const;
	D_COLLISION_API void SetContactNotify(ndSharedPtr<ndContactNotify> notify);

	D_COLLISION_API virtual void DebugScene(ndSceneTreeNotiFy* const notify);

	D_COLLISION_API virtual void BodiesInAabb(ndBodiesInAabbNotify& callback, const ndVector& minBox, const ndVector& maxBox) const;
	D_COLLISION_API virtual bool RayCast(ndRayCastNotify& callback, const ndVector& globalOrigin, const ndVector& globalDest) const;
	D_COLLISION_API virtual bool Collide(ndConvexCastNotify& callback, const ndShapeInstance& instance, const ndMatrix& globalOrigin, ndInt32 maxContacts) const;
	D_COLLISION_API virtual bool ConvexCast(ndConvexCastNotify& callback, const ndShapeInstance& convexShape, const ndMatrix& globalOrigin, const ndVector& globalDest) const;

	D_COLLISION_API ndInt32 GetThreadCount() const;

	D_COLLISION_API virtual ndWorld* GetWorld() const;
	D_COLLISION_API const ndBodyListView& GetBodyList() const;
	D_COLLISION_API const ndBodyList& GetParticleList() const;

	D_COLLISION_API ndArray<ndBodyKinematic*>& GetActiveBodyArray();
	D_COLLISION_API const ndArray<ndBodyKinematic*>& GetActiveBodyArray() const;

	D_COLLISION_API ndArray<ndConstraint*>& GetActiveContactArray();
	D_COLLISION_API const ndArray<ndConstraint*>& GetActiveContactArray() const;

	D_COLLISION_API ndFloat32 GetTimestep() const;
	D_COLLISION_API void SetTimestep(ndFloat32 timestep);
	D_COLLISION_API ndBodyKinematic* GetSentinelBody() const;
	D_COLLISION_API ndArray<ndUnsigned8>& GetScratchBuffer();

	protected:
	D_COLLISION_API ndScene();
	D_COLLISION_API ndScene(const ndScene& src);
	bool ValidateContactCache(ndContact* const contact, const ndVector& timestep) const;
	
	const ndContactArray& GetContactArray() const;
	//void FindCollidingPairs(ndBodyKinematic* const body, ndInt32 threadId);
	void FindCollidingPairsForward(ndBodyKinematic* const body, ndInt32 threadId);
	void FindCollidingPairsBackward(ndBodyKinematic* const body, ndInt32 threadId);
	void AddPair(ndBodyKinematic* const body0, ndBodyKinematic* const body1, ndInt32 threadId);
	void SubmitPairs(ndBvhLeafNode* const bodyNode, ndBvhNode* const node, bool forward, ndInt32 threadId);

	void CalculateJointContacts(ndInt32 threadIndex, ndContact* const contact);
	void ProcessContacts(ndInt32 threadIndex, ndInt32 contactCount, ndContactSolver* const contactSolver);

	ndJointBilateralConstraint* FindBilateralJoint(ndBodyKinematic* const body0, ndBodyKinematic* const body1) const;
	bool RayCast(ndRayCastNotify& callback, ndSceneTraversalStack<const ndBvhNode*>& stackPool, ndSceneTraversalStack<ndFloat32>& distance, const ndFastRay& ray) const;
	bool ConvexCast(ndConvexCastNotify& callback, ndSceneTraversalStack<const ndBvhNode*>& stackPool, ndSceneTraversalStack<ndFloat32>& distance, const ndFastRay& ray, const ndShapeInstance& convexShape, const ndMatrix& globalOrigin, const ndVector& globalDest) const;
	bool Collide(ndConvexCastNotify& callback, ndFixSizeArray<const ndBvhNode*, D_SCENE_MAX_STACK_DEPTH>& stackPool, ndFixSizeArray<ndInt32, D_SCENE_MAX_STACK_DEPTH> overlaped, const ndVector& boxP0, const ndVector& boxP1, const ndShapeInstance& instance, const ndMatrix& matrix, ndInt32 maxContacts) const;

	// call from sub steps update
	D_COLLISION_API virtual void ApplyExtForce();
	D_COLLISION_API virtual void BalanceScene();
	D_COLLISION_API virtual void InitBodyArray();
	D_COLLISION_API virtual void UpdateSpecial();
	D_COLLISION_API virtual void UpdateBodyList();
	D_COLLISION_API virtual void UpdateTransform();
	D_COLLISION_API virtual void CreateNewContacts();
	D_COLLISION_API virtual void CalculateContacts();
	D_COLLISION_API virtual void FindCollidingPairs();
	D_COLLISION_API virtual void DeleteDeadContacts();

	D_COLLISION_API virtual void CalculateContacts(ndInt32 threadIndex, ndContact* const contact);
	//D_COLLISION_API virtual void UpdateTransformNotify(ndInt32 threadIndex, ndBodyKinematic* const body);
	D_COLLISION_API virtual void UpdateTransformNotify(ndFloat32 timestep, ndBodyKinematic* const body);

	D_COLLISION_API virtual void ParticleUpdate(ndFloat32 timestep);
	D_COLLISION_API virtual bool AddParticle(const ndSharedPtr<ndBody>& particle);
	D_COLLISION_API virtual bool RemoveParticle(const ndSharedPtr<ndBody>& particle);

	ndBodyListView m_bodyList;
	ndBodyList m_particleSetList;
	ndContactArray m_contactArray;
	ndBvhSceneManager m_bvhSceneManager;
	ndArray<ndUnsigned8> m_scratchBuffer;
	ndArray<ndBodyKinematic*> m_sceneBodyArray;
	ndArray<ndConstraint*> m_activeConstraintArray;
	ndSpecialList<ndBodyKinematic> m_specialUpdateList;
	ndArray<ndContactPairs> m_newPairs;
	ndArray<ndContactPairs> m_partialNewPairs[D_MAX_THREADS_COUNT];
	ndPolygonMeshDesc::ndStaticMeshFaceQuery m_staticMeshQuery[D_MAX_THREADS_COUNT];
	ndPolygonMeshDesc::ndProceduralStaticMeshFaceQuery m_proceduralStaticMeshQuery[D_MAX_THREADS_COUNT];

	ndSpinLock m_lock;
	ndBvhNode* m_rootNode;
	ndBodyKinematic* m_sentinelBody;
	ndSharedPtr<ndContactNotify> m_contactNotifyCallback;
	
	ndFloat32 m_timestep;
	ndUnsigned32 m_lru;
	ndUnsigned32 m_frameNumber;
	ndUnsigned32 m_subStepNumber;
	ndUnsigned32 m_forceBalanceSceneCounter;

	static ndVector m_velocTol;
	static ndVector m_linearContactError2;
	static ndVector m_angularContactError2;

	friend class ndWorld;
	friend class ndBodyKinematic;
	friend class ndRayCastNotify;
	friend class ndPolygonMeshDesc;
	friend class ndConvexCastNotify;
	friend class ndSkeletonContainer;
	friend class ndConvexCastVehicle;
} D_GCC_NEWTON_CLASS_ALIGN_32 ;


#endif
