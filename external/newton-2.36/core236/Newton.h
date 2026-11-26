/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __NEWTON_H_INCLUDED__
#define __NEWTON_H_INCLUDED__

#include "newton236_platform.h"

#define NEWTON_MAJOR_VERSION 2
#define NEWTON_MINOR_VERSION 36

#define NEWTON_API

#ifdef __USE_DOUBLE_PRECISION__
#define dFloat double
#else
#define dFloat float
#endif

#define dFloat64 double


#ifdef __cplusplus
extern "C" {
#endif

#define NEWTON_PROFILER_WORLD_UPDATE                    0

#define NEWTON_PROFILER_COLLISION_UPDATE                1
#define NEWTON_PROFILER_COLLISION_UPDATE_BROAD_PHASE    2
#define NEWTON_PROFILER_COLLISION_UPDATE_NARROW_PHASE   3

#define NEWTON_PROFILER_DYNAMICS_UPDATE                 4
#define NEWTON_PROFILER_DYNAMICS_CONSTRAINT_GRAPH       5
#define NEWTON_PROFILER_DYNAMICS_SOLVE_CONSTRAINT_GRAPH 6

#define NEWTON_PROFILER_FORCE_CALLBACK_UPDATE           7


class N236_NewtonMesh;
class N236_NewtonBody;
class N236_NewtonWorld;
class N236_NewtonJoint;
class N236_NewtonMaterial;
class N236_NewtonCollision;
class N236_NewtonSceneProxy;
class N236_NewtonbreakableComponentMesh;

//	typedef struct N236_NewtonRagDoll{} N236_NewtonRagDoll;
//	typedef struct N236_NewtonRagDollBone{} N236_NewtonRagDollBone;

#define SERIALIZE_ID_BOX                    0
#define SERIALIZE_ID_CONE                   1
#define SERIALIZE_ID_SPHERE                 2
#define SERIALIZE_ID_CAPSULE                3
#define SERIALIZE_ID_CYLINDER               4
#define SERIALIZE_ID_COMPOUND               5
#define SERIALIZE_ID_CONVEXHULL             6
#define SERIALIZE_ID_CONVEXMODIFIER         7
#define SERIALIZE_ID_CHAMFERCYLINDER        8
#define SERIALIZE_ID_TREE                   9
#define SERIALIZE_ID_NULL                   10
#define SERIALIZE_ID_HEIGHTFIELD            11
#define SERIALIZE_ID_USERMESH               12
#define SERIALIZE_ID_SCENE                  13
#define SERIALIZE_ID_COMPOUND_BREAKABLE     14

typedef struct N236_NewtonBoxParam N236_NewtonBoxParam;
typedef struct N236_NewtonConeParam N236_NewtonConeParam;
typedef struct N236_NewtonSphereParam N236_NewtonSphereParam;
typedef struct N236_NewtonCapsuleParam N236_NewtonCapsuleParam;
typedef struct N236_NewtonCylinderParam N236_NewtonCylinderParam;
typedef struct N236_NewtonConvexHullParam N236_NewtonConvexHullParam;
typedef struct N236_NewtonCollisionTreeParam N236_NewtonCollisionTreeParam;
typedef struct N236_NewtonSceneCollisionParam N236_NewtonSceneCollisionParam;
typedef struct N236_NewtonChamferCylinderParam N236_NewtonChamferCylinderParam;
typedef struct N236_NewtonCompoundCollisionParam N236_NewtonCompoundCollisionParam;
typedef struct N236_NewtonConvexHullModifierParam N236_NewtonConvexHullModifierParam;
typedef struct N236_NewtonHeightFieldCollisionParam N236_NewtonHeightFieldCollisionParam;

typedef struct N236_NewtonCollisionInfoRecord N236_NewtonCollisionInfoRecord;

struct N236_NewtonBoxParam {
	dFloat m_x;
	dFloat m_y;
	dFloat m_z;
};

struct N236_NewtonSphereParam {
	dFloat m_r0;
	dFloat m_r1;
	dFloat m_r2;
};

struct N236_NewtonCylinderParam {
	dFloat m_r0;
	dFloat m_r1;
	dFloat m_height;
};

struct N236_NewtonCapsuleParam {
	dFloat m_r0;
	dFloat m_r1;
	dFloat m_height;
};

struct N236_NewtonConeParam {
	dFloat m_r;
	dFloat m_height;
};

struct N236_NewtonChamferCylinderParam {
	dFloat m_r;
	dFloat m_height;
};

struct N236_NewtonConvexHullParam {
	int m_vertexCount;
	int m_vertexStrideInBytes;
	int m_faceCount;
	dFloat *m_vertex;
};

struct N236_NewtonConvexHullModifierParam {
	N236_NewtonCollision *m_chidren;
};


struct N236_NewtonCompoundCollisionParam {
	int m_chidrenCount;
	N236_NewtonCollision **m_chidren;
};

struct N236_NewtonCollisionTreeParam {
	int m_vertexCount;
	int m_indexCount;
};

struct N236_NewtonHeightFieldCollisionParam {
	int m_width;
	int m_height;
	int m_gridsDiagonals;
	dFloat m_horizonalScale;
	dFloat m_verticalScale;
	unsigned short *m_elevation;
	char *m_atributes;
};

struct N236_NewtonSceneCollisionParam {
	int m_childrenProxyCount;
};

struct N236_NewtonCollisionInfoRecord {
	dFloat m_offsetMatrix[4][4];
	int m_collisionType;              // tag id to identify the collision primitive
	int m_referenceCount;             // the current reference count for this collision
	int m_collisionUserID;

	union {
		N236_NewtonBoxParam m_box;
		N236_NewtonConeParam m_cone;
		N236_NewtonSphereParam m_sphere;
		N236_NewtonCapsuleParam m_capsule;
		N236_NewtonCylinderParam m_cylinder;
		N236_NewtonChamferCylinderParam m_chamferCylinder;
		N236_NewtonConvexHullParam m_convexHull;
		N236_NewtonCompoundCollisionParam m_compoundCollision;
		N236_NewtonConvexHullModifierParam m_convexHullModifier;
		N236_NewtonCollisionTreeParam m_collisionTree;
		N236_NewtonHeightFieldCollisionParam m_heightField;
		N236_NewtonSceneCollisionParam m_sceneCollision;
		dFloat m_paramArray[64];            // user define collision can use this to store information
	};
};


typedef struct N236_NewtonJointRecord N236_NewtonJointRecord;
typedef struct N236_NewtonHingeSliderUpdateDesc N236_NewtonHingeSliderUpdateDesc;
typedef struct N236_NewtonUserMeshCollisionRayHitDesc N236_NewtonUserMeshCollisionRayHitDesc;
typedef struct N236_NewtonUserMeshCollisionCollideDesc N236_NewtonUserMeshCollisionCollideDesc;
typedef struct N236_NewtonWorldConvexCastReturnInfo N236_NewtonWorldConvexCastReturnInfo;


struct N236_NewtonJointRecord {
	dFloat m_attachmenMatrix_0[4][4];
	dFloat m_attachmenMatrix_1[4][4];
	dFloat m_minLinearDof[3];
	dFloat m_maxLinearDof[3];
	dFloat m_minAngularDof[3];
	dFloat m_maxAngularDof[3];
	const N236_NewtonBody *m_attachBody_0;
	const N236_NewtonBody *m_attachBody_1;
	dFloat m_extraParameters[16];
	int   m_bodiesCollisionOn;
	char m_descriptionType[32];
} ;



struct N236_NewtonUserMeshCollisionCollideDesc {
	dFloat m_boxP0[4];                        // lower bounding box of intersection query in local space
	dFloat m_boxP1[4];                        // upper bounding box of intersection query in local space
	int m_threadNumber;                       // current thread executing this query
	int   m_faceCount;                        // the application should set here how many polygons intersect the query box
	int m_vertexStrideInBytes;              // the application should set here the size of each vertex
	void *m_userData;                       // user data passed to the collision geometry at creation time
	dFloat *m_vertex;                       // the application should the pointer to the vertex array.
	int *m_userAttribute;                   // the application should set here the pointer to the user data, one for each face
	int *m_faceIndexCount;                  // the application should set here the pointer to the vertex count of each face.
	int *m_faceVertexIndex;                 // the application should set here the pointer index array for each vertex on a face.
	N236_NewtonBody *m_objBody;                  // pointer to the colliding body
	N236_NewtonBody *m_polySoupBody;             // pointer to the rigid body owner of this collision tree
};

struct N236_NewtonWorldConvexCastReturnInfo {
	dFloat m_point[4];                        // collision point in global space
	dFloat m_normal[4];                       // surface normal at collision point in global space
	dFloat m_normalOnHitPoint[4];           // surface normal at the surface of the hit body,
	// is the same as the normal calculated by a ray cast hitting the body at the hit point
	dFloat m_penetration;                   // contact penetration at collision point
	int    m_contactID;                       // collision ID at contact point
	const  N236_NewtonBody *m_hitBody;         // body hit at contact point
};

struct N236_NewtonUserMeshCollisionRayHitDesc {
	dFloat m_p0[4];                           // ray origin in collision local space
	dFloat m_p1[4];                         // ray destination in collision local space
	dFloat m_normalOut[4];                    // copy here the normal at the ray intersection
	int m_userIdOut;                        // copy here a user defined id for further feedback
	void *m_userData;                       // user data passed to the collision geometry at creation time
};

struct N236_NewtonHingeSliderUpdateDesc {
	dFloat m_accel;
	dFloat m_minFriction;
	dFloat m_maxFriction;
	dFloat m_timestep;
} ;


// N236_Newton callback functions
typedef void *(*N236_NewtonAllocMemory)(int sizeInBytes);
typedef void (*N236_NewtonFreeMemory)(void *const ptr, int sizeInBytes);
typedef void (*N236_NewtonDestroyWorld)(const N236_NewtonWorld *const newtonWorld);

typedef unsigned(*N236_NewtonGetTicksCountCallback)();

typedef void (*N236_NewtonSerialize)(void *const serializeHandle, const void *buffer, size_t size);
typedef void (*N236_NewtonDeserialize)(void *const serializeHandle, void *buffer, size_t size);

// user collision callbacks
typedef void (*N236_NewtonUserMeshCollisionDestroyCallback)(void *const userData);
typedef void (*N236_NewtonUserMeshCollisionCollideCallback)(N236_NewtonUserMeshCollisionCollideDesc *const collideDescData);
typedef dFloat(*N236_NewtonUserMeshCollisionRayHitCallback)(N236_NewtonUserMeshCollisionRayHitDesc *const lineDescData);
typedef void (*N236_NewtonUserMeshCollisionGetCollisionInfo)(void *const userData, N236_NewtonCollisionInfoRecord *const infoRecord);
typedef void (*N236_NewtonUserMeshCollisionGetFacesInAABB)(void *const userData, const dFloat *const p0, const dFloat *const p1,
													  const dFloat *const *const vertexArray, int32 *const vertexCount, int32 *const vertexStrideInBytes,
													  const int32 *const indexList, int32 maxIndexCount, const int32 *const userDataList);

typedef dFloat(*N236_NewtonCollisionTreeRayCastCallback)(const N236_NewtonBody *const body, const N236_NewtonCollision *const treeCollision, dFloat interception, dFloat *normal, int faceId, void *usedData);
typedef dFloat(*N236_NewtonHeightFieldRayCastCallback)(const N236_NewtonBody *const body, const N236_NewtonCollision *const heightFieldCollision, dFloat interception, int row, int col, dFloat *normal, int faceId, void *usedData);


// collision tree call back (obsoleted no recommended)
typedef void (*N236_NewtonTreeCollisionCallback)(const N236_NewtonBody *const bodyWithTreeCollision, const N236_NewtonBody *const body, int faceID,
        int vertexCount, const dFloat *const vertex, int vertexStrideInBytes);

typedef void (*N236_NewtonBodyDestructor)(const N236_NewtonBody *const body);
typedef void (*N236_NewtonApplyForceAndTorque)(N236_NewtonBody *const body, dFloat timestep, int32 threadIndex);
typedef void (*N236_NewtonSetTransform)(const N236_NewtonBody *const body, const dFloat *const matrix, int32 threadIndex);

typedef int (*N236_NewtonIslandUpdate)(const N236_NewtonWorld *const newtonWorld, const void *islandHandle, int bodyCount);
typedef void (*N236_NewtonBodyLeaveWorld)(const N236_NewtonBody *const body, int threadIndex);
typedef void (*N236_NewtonDestroyBodyByExeciveForce)(const N236_NewtonBody *const body, const N236_NewtonJoint *const contact);
typedef void (*N236_NewtonCollisionDestructor)(const N236_NewtonWorld *const newtonWorld, const N236_NewtonCollision *const collision);

typedef int (*N236_NewtonCollisionCompoundBreakableCallback)(N236_NewtonMesh *const mesh, void *const userData, dFloat *const planeMatrixOut);

typedef int (*N236_NewtonGetBuoyancyPlane)(const int32 collisionID, void *const context, const dFloat *const globalSpaceMatrix, dFloat *const globalSpacePlane);
typedef unsigned(*N236_NewtonWorldRayPrefilterCallback)(const N236_NewtonBody *const body, const N236_NewtonCollision *const collision, void *const userData);
typedef dFloat(*N236_NewtonWorldRayFilterCallback)(const N236_NewtonBody *const body, const dFloat *const hitNormal, int collisionID, void *const userData, dFloat intersectParam);


typedef int (*N236_NewtonOnAABBOverlap)(const N236_NewtonMaterial *const material, const N236_NewtonBody *const body0, const N236_NewtonBody *const body1, int32 threadIndex);
typedef void (*N236_NewtonContactsProcess)(const N236_NewtonJoint *const contact, dFloat timestep, int32 threadIndex);

typedef void (*N236_NewtonBodyIterator)(const N236_NewtonBody *const body, void *const userData);
typedef void (*N236_NewtonJointIterator)(const N236_NewtonJoint *const joint, void *const userData);
typedef void (*N236_NewtonCollisionIterator)(void *const userData, int vertexCount, const dFloat *const faceArray, int faceId);

typedef void (*N236_NewtonBallCallBack)(const N236_NewtonJoint *const ball, dFloat timestep);
typedef unsigned(*N236_NewtonHingeCallBack)(const N236_NewtonJoint *const hinge, N236_NewtonHingeSliderUpdateDesc *const desc);
typedef unsigned(*N236_NewtonSliderCallBack)(const N236_NewtonJoint *const slider, N236_NewtonHingeSliderUpdateDesc *const desc);
typedef unsigned(*N236_NewtonUniversalCallBack)(const N236_NewtonJoint *const universal, N236_NewtonHingeSliderUpdateDesc *const desc);
typedef unsigned(*N236_NewtonCorkscrewCallBack)(const N236_NewtonJoint *const corkscrew, N236_NewtonHingeSliderUpdateDesc *const desc);

typedef void (*N236_NewtonUserBilateralCallBack)(const N236_NewtonJoint *const userJoint, dFloat timestep, int threadIndex);
typedef void (*N236_NewtonUserBilateralGetInfoCallBack)(const N236_NewtonJoint *const userJoint, N236_NewtonJointRecord *const info);

typedef void (*N236_NewtonConstraintDestructor)(const N236_NewtonJoint *const  me);

//	typedef void (*N236_NewtonSetRagDollTransform) (const N236_NewtonRagDollBone* const bone);
//	typedef void (*N236_NewtonBodyActivationState) (const N236_NewtonBody* const body, unsigned state);
//	typedef void (*N236_NewtonVehicleTireUpdate) (const N236_NewtonJoint* const vehicle, dFloat timestep);


void N236_NewtonInitGlobals();

void N236_NewtonDestroyGlobals();

// **********************************************************************************************
//
// world control functions
//
// **********************************************************************************************
NEWTON_API int N236_NewtonWorldGetVersion();
NEWTON_API int N236_NewtonWorldFloatSize();

NEWTON_API int N236_NewtonGetMemoryUsed();
NEWTON_API void N236_NewtonSetMemorySystem(N236_NewtonAllocMemory malloc, N236_NewtonFreeMemory mfree);

NEWTON_API N236_NewtonWorld *N236_NewtonCreate();
NEWTON_API void N236_NewtonDestroy(const N236_NewtonWorld *const newtonWorld);
NEWTON_API void N236_NewtonDestroyAllBodies(const N236_NewtonWorld *const newtonWorld);

NEWTON_API void N236_NewtonUpdate(N236_NewtonWorld *const newtonWorld, dFloat timestep);
NEWTON_API void N236_NewtonInvalidateCache(N236_NewtonWorld *const newtonWorld);
NEWTON_API void N236_NewtonCollisionUpdate(N236_NewtonWorld *const newtonWorld);

NEWTON_API void N236_NewtonSetSolverModel(N236_NewtonWorld *const newtonWorld, int model);
NEWTON_API void N236_NewtonSetPlatformArchitecture(N236_NewtonWorld *const newtonWorld, int mode);
NEWTON_API int N236_NewtonGetPlatformArchitecture(const N236_NewtonWorld *const newtonWorld, char *description);
NEWTON_API void N236_NewtonSetMultiThreadSolverOnSingleIsland(N236_NewtonWorld *const newtonWorld, int mode);
NEWTON_API int N236_NewtonGetMultiThreadSolverOnSingleIsland(const N236_NewtonWorld *const newtonWorld);

NEWTON_API void N236_NewtonSetPerformanceClock(N236_NewtonWorld *const newtonWorld, N236_NewtonGetTicksCountCallback callback);
NEWTON_API unsigned N236_NewtonReadPerformanceTicks(const N236_NewtonWorld *const newtonWorld, unsigned performanceEntry);
NEWTON_API unsigned N236_NewtonReadThreadPerformanceTicks(const N236_NewtonWorld *newtonWorld, unsigned threadIndex);

NEWTON_API void N236_NewtonWorldCriticalSectionLock(N236_NewtonWorld *const newtonWorld);
NEWTON_API void N236_NewtonWorldCriticalSectionUnlock(N236_NewtonWorld *const newtonWorld);
NEWTON_API void N236_NewtonSetThreadsCount(N236_NewtonWorld *const newtonWorld, int threads);
NEWTON_API int N236_NewtonGetThreadsCount(const N236_NewtonWorld *const newtonWorld);
NEWTON_API int N236_NewtonGetMaxThreadsCount(const N236_NewtonWorld *const newtonWorld);

NEWTON_API void N236_NewtonSetFrictionModel(N236_NewtonWorld *const newtonWorld, int model);
NEWTON_API void N236_NewtonSetMinimumFrameRate(N236_NewtonWorld *const newtonWorld, dFloat frameRate);
NEWTON_API void N236_NewtonSetBodyLeaveWorldEvent(N236_NewtonWorld *const newtonWorld, N236_NewtonBodyLeaveWorld callback);
NEWTON_API void N236_NewtonSetWorldSize(N236_NewtonWorld *const newtonWorld, const dFloat *const minPoint, const dFloat *const maxPoint);
NEWTON_API void N236_NewtonSetIslandUpdateEvent(N236_NewtonWorld *const newtonWorld, N236_NewtonIslandUpdate islandUpdate);
NEWTON_API void N236_NewtonSetCollisionDestructor(N236_NewtonWorld *const newtonWorld, N236_NewtonCollisionDestructor callback);
NEWTON_API void N236_NewtonSetDestroyBodyByExeciveForce(N236_NewtonWorld *const newtonWorld, N236_NewtonDestroyBodyByExeciveForce callback);


//	NEWTON_API void N236_NewtonWorldForEachBodyDo (const N236_NewtonWorld* const newtonWorld, N236_NewtonBodyIterator callback);
NEWTON_API void N236_NewtonWorldForEachJointDo(const N236_NewtonWorld *const newtonWorld, N236_NewtonJointIterator callback, void *const userData);
NEWTON_API void N236_NewtonWorldForEachBodyInAABBDo(const N236_NewtonWorld *const newtonWorld, const dFloat *const p0, const dFloat *const p1, N236_NewtonBodyIterator callback, void *const userData);


NEWTON_API void N236_NewtonWorldSetUserData(N236_NewtonWorld *const newtonWorld, void *const userData);
NEWTON_API void *N236_NewtonWorldGetUserData(const N236_NewtonWorld *const newtonWorld);
NEWTON_API void N236_NewtonWorldSetDestructorCallBack(N236_NewtonWorld *const newtonWorld, N236_NewtonDestroyWorld destructor);
NEWTON_API N236_NewtonDestroyWorld N236_NewtonWorldGetDestructorCallBack(const N236_NewtonWorld *const newtonWorld);

NEWTON_API void N236_NewtonWorldRayCast(N236_NewtonWorld *const newtonWorld, const dFloat *const p0, const dFloat *const p1, N236_NewtonWorldRayFilterCallback filter, void *const userData,
                                   N236_NewtonWorldRayPrefilterCallback prefilter);
NEWTON_API int N236_NewtonWorldConvexCast(N236_NewtonWorld *const newtonWorld, const dFloat *const matrix, const dFloat *const target, N236_NewtonCollision *shape, dFloat *const hitParam, void *const userData,
                                     N236_NewtonWorldRayPrefilterCallback prefilter, N236_NewtonWorldConvexCastReturnInfo *info, int maxContactsCount, int threadIndex);


// world utility functions
NEWTON_API int N236_NewtonWorldGetBodyCount(const N236_NewtonWorld *const newtonWorld);
NEWTON_API int N236_NewtonWorldGetConstraintCount(const N236_NewtonWorld *const newtonWorld);

// NEWTON_API int N236_NewtonGetActiveBodiesCount();
// NEWTON_API int N236_NewtonGetActiveConstraintsCount();
// NEWTON_API dFloat N236_NewtonGetGlobalScale (const N236_NewtonWorld* const newtonWorld);



// **********************************************************************************************
//
// Simulation islands
//
// **********************************************************************************************
NEWTON_API N236_NewtonBody *N236_NewtonIslandGetBody(const void *const island, int bodyIndex);
NEWTON_API void N236_NewtonIslandGetBodyAABB(const void *const island, int bodyIndex, dFloat *const p0, dFloat *const p1);

// **********************************************************************************************
//
// Physics Material Section
//
// **********************************************************************************************
NEWTON_API int N236_NewtonMaterialCreateGroupID(N236_NewtonWorld *const newtonWorld);
NEWTON_API int N236_NewtonMaterialGetDefaultGroupID(const N236_NewtonWorld *const newtonWorld);
NEWTON_API void N236_NewtonMaterialDestroyAllGroupID(N236_NewtonWorld *const newtonWorld);

// material definitions that can not be overwritten in function callback
NEWTON_API void *N236_NewtonMaterialGetUserData(const N236_NewtonWorld *const newtonWorld, int id0, int id1);
NEWTON_API void N236_NewtonMaterialSetSurfaceThickness(const N236_NewtonWorld *const newtonWorld, int id0, int id1, dFloat thickness);
NEWTON_API void N236_NewtonMaterialSetContinuousCollisionMode(const N236_NewtonWorld *const newtonWorld, int id0, int id1, int state);
NEWTON_API void N236_NewtonMaterialSetCollisionCallback(const N236_NewtonWorld *const newtonWorld, int id0, int id1, void *const userData,
        N236_NewtonOnAABBOverlap aabbOverlap, N236_NewtonContactsProcess process);

NEWTON_API void N236_NewtonMaterialSetDefaultSoftness(const N236_NewtonWorld *const newtonWorld, int id0, int id1, dFloat value);
NEWTON_API void N236_NewtonMaterialSetDefaultElasticity(const N236_NewtonWorld *const newtonWorld, int id0, int id1, dFloat elasticCoef);
NEWTON_API void N236_NewtonMaterialSetDefaultCollidable(const N236_NewtonWorld *const newtonWorld, int id0, int id1, int state);
NEWTON_API void N236_NewtonMaterialSetDefaultFriction(const N236_NewtonWorld *const newtonWorld, int id0, int id1, dFloat staticFriction, dFloat kineticFriction);

NEWTON_API N236_NewtonMaterial *N236_NewtonWorldGetFirstMaterial(const N236_NewtonWorld *const newtonWorld);
NEWTON_API N236_NewtonMaterial *N236_NewtonWorldGetNextMaterial(const N236_NewtonWorld *const newtonWorld, N236_NewtonMaterial *const material);

NEWTON_API N236_NewtonBody *N236_NewtonWorldGetFirstBody(const N236_NewtonWorld *const newtonWorld);
NEWTON_API N236_NewtonBody *N236_NewtonWorldGetNextBody(const N236_NewtonWorld *const newtonWorld, const N236_NewtonBody *const curBody);


// **********************************************************************************************
//
// Physics Contact control functions
//
// **********************************************************************************************
NEWTON_API void *N236_NewtonMaterialGetMaterialPairUserData(const N236_NewtonMaterial *const material);
NEWTON_API unsigned N236_NewtonMaterialGetContactFaceAttribute(const N236_NewtonMaterial *const material);
NEWTON_API N236_NewtonCollision *N236_NewtonMaterialGetBodyCollidingShape(const N236_NewtonMaterial *const material, const N236_NewtonBody *const body);
//NEWTON_API unsigned N236_NewtonMaterialGetBodyCollisionID (const N236_NewtonMaterial* const material, const N236_NewtonBody* const body);
NEWTON_API dFloat N236_NewtonMaterialGetContactNormalSpeed(const N236_NewtonMaterial *const material);
NEWTON_API void N236_NewtonMaterialGetContactForce(const N236_NewtonMaterial *const material, N236_NewtonBody *const body, dFloat *const force);
NEWTON_API void N236_NewtonMaterialGetContactPositionAndNormal(const N236_NewtonMaterial *const material, N236_NewtonBody *const body, dFloat *const posit, dFloat *const normal);
NEWTON_API void N236_NewtonMaterialGetContactTangentDirections(const N236_NewtonMaterial *const material, N236_NewtonBody *const body, dFloat *const dir0, dFloat *const dir1);
NEWTON_API dFloat N236_NewtonMaterialGetContactTangentSpeed(const N236_NewtonMaterial *const material, int index);

NEWTON_API void N236_NewtonMaterialSetContactSoftness(N236_NewtonMaterial *const material, dFloat softness);
NEWTON_API void N236_NewtonMaterialSetContactElasticity(N236_NewtonMaterial *const material, dFloat restitution);
NEWTON_API void N236_NewtonMaterialSetContactFrictionState(N236_NewtonMaterial *const material, int state, int index);
NEWTON_API void N236_NewtonMaterialSetContactFrictionCoef(N236_NewtonMaterial *const material, dFloat staticFrictionCoef, dFloat kineticFrictionCoef, int index);

NEWTON_API void N236_NewtonMaterialSetContactNormalAcceleration(N236_NewtonMaterial *const material, dFloat accel);
NEWTON_API void N236_NewtonMaterialSetContactNormalDirection(N236_NewtonMaterial *const material, const dFloat *const directionVector);

NEWTON_API void N236_NewtonMaterialSetContactTangentAcceleration(N236_NewtonMaterial *const material, dFloat accel, int index);
NEWTON_API void N236_NewtonMaterialContactRotateTangentDirections(N236_NewtonMaterial *const material, const dFloat *const directionVector);



// **********************************************************************************************
//
// convex collision primitives creation functions
//
// **********************************************************************************************
NEWTON_API N236_NewtonCollision *N236_NewtonCreateNull(N236_NewtonWorld *const newtonWorld);
NEWTON_API N236_NewtonCollision *N236_NewtonCreateSphere(N236_NewtonWorld *const newtonWorld, dFloat radiusX, dFloat radiusY, dFloat radiusZ, int shapeID, const dFloat *const offsetMatrix);
NEWTON_API N236_NewtonCollision *N236_NewtonCreateBox(N236_NewtonWorld *const newtonWorld, dFloat dx, dFloat dy, dFloat dz, int shapeID, const dFloat *const offsetMatrix);
NEWTON_API N236_NewtonCollision *N236_NewtonCreateCone(N236_NewtonWorld *const newtonWorld, dFloat radius, dFloat height, int shapeID, const dFloat *const offsetMatrix);
NEWTON_API N236_NewtonCollision *N236_NewtonCreateCapsule(N236_NewtonWorld *const newtonWorld, dFloat radius, dFloat height, int shapeID, const dFloat *const offsetMatrix);
NEWTON_API N236_NewtonCollision *N236_NewtonCreateCylinder(N236_NewtonWorld *const newtonWorld, dFloat radius, dFloat height, int shapeID, const dFloat *const offsetMatrix);
NEWTON_API N236_NewtonCollision *N236_NewtonCreateChamferCylinder(N236_NewtonWorld *const newtonWorld, dFloat radius, dFloat height, int shapeID, const dFloat *const offsetMatrix);
NEWTON_API N236_NewtonCollision *N236_NewtonCreateConvexHull(N236_NewtonWorld *const newtonWorld, int count, const dFloat *const vertexCloud, int strideInBytes, dFloat tolerance, int shapeID, const dFloat *const offsetMatrix);
NEWTON_API N236_NewtonCollision *N236_NewtonCreateConvexHullFromMesh(const N236_NewtonWorld *const newtonWorld, const N236_NewtonMesh *const mesh, dFloat tolerance, int shapeID);

NEWTON_API N236_NewtonCollision *N236_NewtonCreateConvexHullModifier(N236_NewtonWorld *const newtonWorld, N236_NewtonCollision *const convexHullCollision, int shapeID);
NEWTON_API void N236_NewtonConvexHullModifierGetMatrix(const N236_NewtonCollision *const convexHullCollision, dFloat *const matrix);
NEWTON_API void N236_NewtonConvexHullModifierSetMatrix(N236_NewtonCollision *const convexHullCollision, const dFloat *const matrix);

NEWTON_API int N236_NewtonCollisionIsTriggerVolume(const N236_NewtonCollision *const convexCollision);
NEWTON_API void N236_NewtonCollisionSetAsTriggerVolume(N236_NewtonCollision *convexCollision, int trigger);

NEWTON_API void N236_NewtonCollisionSetMaxBreakImpactImpulse(N236_NewtonCollision *const convexHullCollision, dFloat maxImpactImpulse);
NEWTON_API dFloat N236_NewtonCollisionGetMaxBreakImpactImpulse(const N236_NewtonCollision *const convexHullCollision);

NEWTON_API void N236_NewtonCollisionSetUserID(N236_NewtonCollision *convexCollision, unsigned id);
NEWTON_API unsigned N236_NewtonCollisionGetUserID(N236_NewtonCollision *const convexCollision);

NEWTON_API int N236_NewtonConvexHullGetFaceIndices(const N236_NewtonCollision *const convexHullCollision, int face, int32 *const faceIndices);
NEWTON_API dFloat N236_NewtonConvexCollisionCalculateVolume(const N236_NewtonCollision *const convexCollision);
NEWTON_API void N236_NewtonConvexCollisionCalculateInertialMatrix(const N236_NewtonCollision *convexCollision, dFloat *const inertia, dFloat *const origin);


NEWTON_API void N236_NewtonCollisionMakeUnique(N236_NewtonWorld *const newtonWorld, N236_NewtonCollision *const collision);
NEWTON_API void N236_NewtonReleaseCollision(N236_NewtonWorld *const newtonWorld, N236_NewtonCollision *const collision);
NEWTON_API int N236_NewtonAddCollisionReference(N236_NewtonCollision *const collision);


// **********************************************************************************************
//
// mass/spring/damper collision shape
//
// **********************************************************************************************
//	NEWTON_API N236_NewtonCollision* N236_NewtonCreateSoftShape (const N236_NewtonWorld* const newtonWorld);
//	NEWTON_API void N236_NewtonSoftBodySetMassCount (const N236_NewtonCollision* convexCollision, int count);
//	NEWTON_API void N236_NewtonSoftBodySetSpringCount (const N236_NewtonCollision* convexCollision, int count);

//	NEWTON_API void N236_NewtonSoftBodySetMass (const N236_NewtonCollision* convexCollision, int index, dFloat mass, dFloat* position);
//	NEWTON_API int N236_NewtonSoftBodySetSpring (const N236_NewtonCollision* convexCollision, int index, int mass0, int mass1, dFloat stiffness, dFloat damper);
//	NEWTON_API int N236_NewtonSoftBodyGetMassArray (const N236_NewtonCollision* convexCollision, dFloat* masses, dFloat** positions);


// **********************************************************************************************
//
// complex collision primitives creation functions
//
// **********************************************************************************************
NEWTON_API N236_NewtonCollision *N236_NewtonCreateCompoundCollision(N236_NewtonWorld *const newtonWorld, int count, N236_NewtonCollision *const collisionPrimitiveArray[], int shapeID);
NEWTON_API N236_NewtonCollision *N236_NewtonCreateCompoundCollisionFromMesh(N236_NewtonWorld *const newtonWorld, const N236_NewtonMesh *const mesh, dFloat hullTolerance, int shapeID, int subShapeID);
//NEWTON_API N236_NewtonCollision* N236_NewtonCreateCompoundCollisionFromMesh(const N236_NewtonWorld* const newtonWorld, const N236_NewtonMesh* const mesh, int maxSubShapesCount, int shapeID, int subShapeID);

// **********************************************************************************************
//
// complex breakable collision primitives interface
//
// **********************************************************************************************
//	NEWTON_API N236_NewtonCollision* N236_NewtonCreateCompoundBreakable (const N236_NewtonWorld* const newtonWorld, int meshCount,
//															   N236_NewtonMesh* const solids[], N236_NewtonMesh* const splitePlanes[],
//															   dFloat* const matrixPallete, int32* const shapeIDArray, dFloat* const densities,
//															   int shapeID, int debriID, N236_NewtonCollisionCompoundBreakableCallback callback, void* buildUsedData);

NEWTON_API N236_NewtonCollision *N236_NewtonCreateCompoundBreakable(N236_NewtonWorld *const newtonWorld, int meshCount,
        N236_NewtonMesh **const solids, const int32 *const shapeIDArray,
        const dFloat *const densities, const int32 *const internalFaceMaterial,
        int shapeID, int debriID, dFloat debriSeparationGap);


NEWTON_API void N236_NewtonCompoundBreakableResetAnchoredPieces(N236_NewtonCollision *const compoundBreakable);
NEWTON_API void N236_NewtonCompoundBreakableSetAnchoredPieces(N236_NewtonCollision *const compoundBreakable, int fixShapesCount, dFloat *const matrixPallete, N236_NewtonCollision **const fixedShapesArray);

NEWTON_API int N236_NewtonCompoundBreakableGetVertexCount(const N236_NewtonCollision *const compoundBreakable);
NEWTON_API void N236_NewtonCompoundBreakableGetVertexStreams(const N236_NewtonCollision *const compoundBreakable, int vertexStrideInByte, dFloat *const vertex,
        int normalStrideInByte, dFloat *const normal,  int uvStrideInByte, dFloat *const uv);


NEWTON_API N236_NewtonbreakableComponentMesh *N236_NewtonBreakableGetMainMesh(const N236_NewtonCollision *const compoundBreakable);
NEWTON_API N236_NewtonbreakableComponentMesh *N236_NewtonBreakableGetFirstComponent(const N236_NewtonCollision *const compoundBreakable);
NEWTON_API const N236_NewtonbreakableComponentMesh *N236_NewtonBreakableGetNextComponent(const N236_NewtonbreakableComponentMesh *const component);

NEWTON_API void N236_NewtonBreakableBeginDelete(N236_NewtonCollision *const compoundBreakable);
NEWTON_API N236_NewtonBody *N236_NewtonBreakableCreateDebrieBody(N236_NewtonCollision *const compoundBreakable, N236_NewtonbreakableComponentMesh *const component);
NEWTON_API void N236_NewtonBreakableDeleteComponent(N236_NewtonCollision *const compoundBreakable, N236_NewtonbreakableComponentMesh *const component);
NEWTON_API void N236_NewtonBreakableEndDelete(N236_NewtonCollision *const compoundBreakable);


NEWTON_API int N236_NewtonBreakableGetComponentsInRadius(const N236_NewtonCollision *const compoundBreakable, const dFloat *position, dFloat radius, N236_NewtonbreakableComponentMesh **const segments, int maxCount);

NEWTON_API void *N236_NewtonBreakableGetFirstSegment(N236_NewtonbreakableComponentMesh *const breakableComponent);
NEWTON_API void *N236_NewtonBreakableGetNextSegment(const void *const segment);

NEWTON_API int N236_NewtonBreakableSegmentGetMaterial(void *const segment);
NEWTON_API int N236_NewtonBreakableSegmentGetIndexCount(void *const segment);
NEWTON_API int N236_NewtonBreakableSegmentGetIndexStream(const N236_NewtonCollision *const compoundBreakable, N236_NewtonbreakableComponentMesh *const meshOwner, void *const segment, int32 *const index);
NEWTON_API int N236_NewtonBreakableSegmentGetIndexStreamShort(const N236_NewtonCollision *const compoundBreakable, const N236_NewtonbreakableComponentMesh *const meshOwner, void *const segment, short int *const index);


NEWTON_API N236_NewtonCollision *N236_NewtonCreateUserMeshCollision(N236_NewtonWorld *const newtonWorld, const dFloat *const minBox,
        const dFloat *const maxBox, void *const userData, N236_NewtonUserMeshCollisionCollideCallback collideCallback,
        N236_NewtonUserMeshCollisionRayHitCallback rayHitCallback, N236_NewtonUserMeshCollisionDestroyCallback destroyCallback,
        N236_NewtonUserMeshCollisionGetCollisionInfo getInfoCallback, N236_NewtonUserMeshCollisionGetFacesInAABB facesInAABBCallback, int shapeID);


NEWTON_API N236_NewtonCollision *N236_NewtonCreateSceneCollision(N236_NewtonWorld *const newtonWorld, int shapeID);
NEWTON_API N236_NewtonSceneProxy *N236_NewtonSceneCollisionCreateProxy(N236_NewtonCollision *const scene, N236_NewtonCollision *const collision, dFloat *const matrix);

NEWTON_API void N236_NewtonSceneCollisionDestroyProxy(N236_NewtonCollision *const scene, N236_NewtonSceneProxy *Proxy);
NEWTON_API void N236_NewtonSceneProxySetMatrix(N236_NewtonSceneProxy *const proxy, const dFloat *matrix);
NEWTON_API void N236_NewtonSceneProxyGetMatrix(N236_NewtonSceneProxy *const proxy, dFloat *matrix);
NEWTON_API void N236_NewtonSceneSetProxyUserData(N236_NewtonSceneProxy *const proxy, void *userData);
NEWTON_API void *N236_NewtonSceneGetProxyUserData(N236_NewtonSceneProxy *const proxy);

NEWTON_API N236_NewtonSceneProxy *N236_NewtonSceneGetFirstProxy(N236_NewtonCollision *const scene);
NEWTON_API N236_NewtonSceneProxy *N236_NewtonSceneGetNextProxy(N236_NewtonCollision *const scene, N236_NewtonSceneProxy *const proxy);

NEWTON_API void N236_NewtonSceneCollisionOptimize(N236_NewtonCollision *scene);

//  ***********************************************************************************************************
//
//	Collision serialization functions
//
// ***********************************************************************************************************
NEWTON_API N236_NewtonCollision *N236_NewtonCreateCollisionFromSerialization(N236_NewtonWorld *const newtonWorld, N236_NewtonDeserialize deserializeFunction, void *const serializeHandle);
NEWTON_API void N236_NewtonCollisionSerialize(const N236_NewtonWorld *const newtonWorld, const N236_NewtonCollision *const collision, N236_NewtonSerialize serializeFunction, void *const serializeHandle);
NEWTON_API void N236_NewtonCollisionGetInfo(const N236_NewtonCollision *const collision, N236_NewtonCollisionInfoRecord *const collisionInfo);

// **********************************************************************************************
//
// Static collision shapes functions
//
// **********************************************************************************************
NEWTON_API N236_NewtonCollision *N236_NewtonCreateHeightFieldCollision(N236_NewtonWorld *const newtonWorld, int width, int height, int gridsDiagonals,
        const unsigned short *const elevationMap, const int8 *const attributeMap,
        dFloat horizontalScale, dFloat verticalScale, int shapeID);
NEWTON_API void N236_NewtonHeightFieldSetUserRayCastCallback(N236_NewtonCollision *const treeCollision, N236_NewtonHeightFieldRayCastCallback rayHitCallback);


NEWTON_API N236_NewtonCollision *N236_NewtonCreateTreeCollision(N236_NewtonWorld *const newtonWorld, int shapeID);
NEWTON_API N236_NewtonCollision *N236_NewtonCreateTreeCollisionFromMesh(const N236_NewtonWorld *const newtonWorld, const N236_NewtonMesh *const mesh, int shapeID);

NEWTON_API void N236_NewtonTreeCollisionSetUserRayCastCallback(const N236_NewtonCollision *const treeCollision, N236_NewtonCollisionTreeRayCastCallback rayHitCallback);

NEWTON_API void N236_NewtonTreeCollisionBeginBuild(N236_NewtonCollision *treeCollision);
NEWTON_API void N236_NewtonTreeCollisionAddFace(N236_NewtonCollision *const treeCollision, int vertexCount, const dFloat *const vertexPtr, int strideInBytes, int faceAttribute);
NEWTON_API void N236_NewtonTreeCollisionEndBuild(N236_NewtonCollision *const treeCollision, int optimize);

NEWTON_API int N236_NewtonTreeCollisionGetFaceAtribute(const N236_NewtonCollision *const treeCollision, const int32 *const faceIndexArray);
NEWTON_API void N236_NewtonTreeCollisionSetFaceAtribute(N236_NewtonCollision *const treeCollision, const int32 *const faceIndexArray, int attribute);
NEWTON_API int N236_NewtonTreeCollisionGetVertexListIndexListInAABB(const N236_NewtonCollision *const treeCollision, const dFloat *const p0, const dFloat *const p1, const dFloat **const vertexArray, int32 *const vertexCount, int32 *const vertexStrideInBytes, int32 *const indexList, int maxIndexCount, int32 *const faceAttribute);


NEWTON_API void N236_NewtonStaticCollisionSetDebugCallback(N236_NewtonCollision *const staticCollision, N236_NewtonTreeCollisionCallback userCallback);

// **********************************************************************************************
//
// General purpose collision library functions
//
// **********************************************************************************************
NEWTON_API int N236_NewtonCollisionPointDistance(N236_NewtonWorld *const newtonWorld, const dFloat *const point,
        N236_NewtonCollision *const collision, const dFloat *const matrix, dFloat *const contact, dFloat *const normal, int threadIndex);

NEWTON_API int N236_NewtonCollisionClosestPoint(N236_NewtonWorld *const newtonWorld,
        N236_NewtonCollision *const collisionA, const dFloat *const matrixA, N236_NewtonCollision *const collisionB, const dFloat *const matrixB,
        dFloat *const contactA, dFloat *const contactB, dFloat *const normalAB, int threadIndex);

NEWTON_API int N236_NewtonCollisionCollide(N236_NewtonWorld *const newtonWorld, int maxSize,
                                      N236_NewtonCollision *const collisionA, const dFloat *const matrixA,
                                      N236_NewtonCollision *const collisionB, const dFloat *const matrixB,
                                      dFloat *const contacts, dFloat *const normals, dFloat *const penetration, int threadIndex);

NEWTON_API int N236_NewtonCollisionCollideContinue(N236_NewtonWorld *const newtonWorld, int maxSize, const dFloat timestep,
        N236_NewtonCollision *const collisionA, const dFloat *const matrixA, const dFloat *const velocA, const dFloat *omegaA,
        N236_NewtonCollision *const collisionB, const dFloat *const matrixB, const dFloat *const velocB, const dFloat *const omegaB,
        dFloat *const timeOfImpact, dFloat *const contacts, dFloat *const normals, dFloat *const penetration, int threadIndex);

NEWTON_API void N236_NewtonCollisionSupportVertex(const N236_NewtonCollision *const collision, const dFloat *const dir, dFloat *const vertex);
NEWTON_API dFloat N236_NewtonCollisionRayCast(const N236_NewtonCollision *const collision, const dFloat *const p0, const dFloat *const p1, dFloat *const normal, int *const attribute);
NEWTON_API void N236_NewtonCollisionCalculateAABB(const N236_NewtonCollision *const collision, const dFloat *const matrix, dFloat *const p0, dFloat *const p1);
NEWTON_API void N236_NewtonCollisionForEachPolygonDo(const N236_NewtonCollision *const collision, const dFloat *const matrix, N236_NewtonCollisionIterator callback, void *const userData);


// **********************************************************************************************
//
// transforms utility functions
//
// **********************************************************************************************
NEWTON_API void N236_NewtonGetEulerAngle(const dFloat *const matrix, dFloat *const eulersAngles);
NEWTON_API void N236_NewtonSetEulerAngle(const dFloat *const eulersAngles, dFloat *const matrix);
NEWTON_API dFloat N236_NewtonCalculateSpringDamperAcceleration(dFloat dt, dFloat ks, dFloat x, dFloat kd, dFloat s);

// **********************************************************************************************
//
// body manipulation functions
//
// **********************************************************************************************
NEWTON_API N236_NewtonBody *N236_NewtonCreateBody(N236_NewtonWorld *const newtonWorld, N236_NewtonCollision *const collision, const dFloat *const matrix);
NEWTON_API void  N236_NewtonDestroyBody(N236_NewtonWorld *const newtonWorld, N236_NewtonBody *const body);

NEWTON_API void  N236_NewtonBodyAddForce(N236_NewtonBody *const body, const dFloat *const force);
NEWTON_API void  N236_NewtonBodyAddTorque(N236_NewtonBody *const body, const dFloat *const torque);
NEWTON_API void  N236_NewtonBodyCalculateInverseDynamicsForce(const N236_NewtonBody *const body, dFloat timestep, const dFloat *const desiredVeloc, dFloat *const forceOut);

NEWTON_API void  N236_NewtonBodySetMatrix(N236_NewtonBody *const body, dFloat *const matrix);
NEWTON_API void  N236_NewtonBodySetMatrixRecursive(N236_NewtonBody *const body, const dFloat *const matrix);
NEWTON_API void  N236_NewtonBodySetMassMatrix(N236_NewtonBody *const body, dFloat mass, dFloat Ixx, dFloat Iyy, dFloat Izz);
NEWTON_API void  N236_NewtonBodySetMaterialGroupID(N236_NewtonBody *body, int id);
NEWTON_API void  N236_NewtonBodySetContinuousCollisionMode(N236_NewtonBody *body, unsigned state);
NEWTON_API void  N236_NewtonBodySetJointRecursiveCollision(N236_NewtonBody *body, unsigned state);
NEWTON_API void  N236_NewtonBodySetOmega(N236_NewtonBody *body, const dFloat *const omega);
NEWTON_API void  N236_NewtonBodySetVelocity(N236_NewtonBody *body, const dFloat *const velocity);
NEWTON_API void  N236_NewtonBodySetForce(N236_NewtonBody *const body, const dFloat *const force);
NEWTON_API void  N236_NewtonBodySetTorque(N236_NewtonBody *body, const dFloat *const torque);

NEWTON_API void  N236_NewtonBodySetCentreOfMass(N236_NewtonBody *body, const dFloat *const com);
NEWTON_API void  N236_NewtonBodySetLinearDamping(N236_NewtonBody *body, dFloat linearDamp);
NEWTON_API void  N236_NewtonBodySetAngularDamping(N236_NewtonBody *body, const dFloat *const angularDamp);
NEWTON_API void  N236_NewtonBodySetUserData(N236_NewtonBody *const body, void *const userData);
NEWTON_API void  N236_NewtonBodySetCollision(N236_NewtonBody *body, N236_NewtonCollision *collision);



NEWTON_API int  N236_NewtonBodyGetSleepState(const N236_NewtonBody *const body);
NEWTON_API int  N236_NewtonBodyGetAutoSleep(const N236_NewtonBody *const body);
NEWTON_API void N236_NewtonBodySetAutoSleep(N236_NewtonBody *body, int state);

NEWTON_API int  N236_NewtonBodyGetFreezeState(const N236_NewtonBody *const body);
NEWTON_API void N236_NewtonBodySetFreezeState(N236_NewtonBody *body, int state);


//	NEWTON_API void N236_NewtonBodySetAutoFreeze(const N236_NewtonBody* const body, int state);
//	NEWTON_API void N236_NewtonBodyCoriolisForcesMode (const N236_NewtonBody* const body, int mode);
//	NEWTON_API void N236_NewtonBodySetGyroscopicForcesMode (const N236_NewtonBody* const body, int mode);
//	NEWTON_API int  N236_NewtonBodyGetGyroscopicForcesMode (const N236_NewtonBody* const body);
//	NEWTON_API int  N236_NewtonBodyGetFreezeState (const N236_NewtonBody* const body);
//	NEWTON_API void N236_NewtonBodySetFreezeState  (const N236_NewtonBody* const body, int state);
//	NEWTON_API void N236_NewtonBodyGetFreezeTreshold (const N236_NewtonBody* const body, dFloat* freezeSpeed2, dFloat* freezeOmega2);
//	NEWTON_API void N236_NewtonBodySetFreezeTreshold (const N236_NewtonBody* const body, dFloat freezeSpeed2, dFloat freezeOmega2, int framesCount);
//	NEWTON_API void N236_NewtonBodySetAutoactiveCallback (const N236_NewtonBody* const body, N236_NewtonBodyActivationState callback);


NEWTON_API void  N236_NewtonBodySetDestructorCallback(N236_NewtonBody *const body, N236_NewtonBodyDestructor callback);

NEWTON_API void  N236_NewtonBodySetTransformCallback(N236_NewtonBody *const body, N236_NewtonSetTransform callback);
NEWTON_API N236_NewtonSetTransform N236_NewtonBodyGetTransformCallback(const N236_NewtonBody *const body);

NEWTON_API void  N236_NewtonBodySetForceAndTorqueCallback(N236_NewtonBody *const body, N236_NewtonApplyForceAndTorque callback);
NEWTON_API N236_NewtonApplyForceAndTorque N236_NewtonBodyGetForceAndTorqueCallback(const N236_NewtonBody *const body);
NEWTON_API void *N236_NewtonBodyGetUserData(const N236_NewtonBody *const body);
NEWTON_API N236_NewtonWorld *N236_NewtonBodyGetWorld(const N236_NewtonBody *const body);
NEWTON_API N236_NewtonCollision *N236_NewtonBodyGetCollision(const N236_NewtonBody *const body);
NEWTON_API int   N236_NewtonBodyGetMaterialGroupID(const N236_NewtonBody *const body);

NEWTON_API int   N236_NewtonBodyGetContinuousCollisionMode(const N236_NewtonBody *const body);
NEWTON_API int   N236_NewtonBodyGetJointRecursiveCollision(const N236_NewtonBody *const body);

NEWTON_API void  N236_NewtonBodyGetMatrix(const N236_NewtonBody *const body, dFloat *const matrix);
NEWTON_API void  N236_NewtonBodyGetRotation(const N236_NewtonBody *const body, dFloat *const rotation);
NEWTON_API void  N236_NewtonBodyGetMassMatrix(const N236_NewtonBody *const body, dFloat *mass, dFloat *const Ixx, dFloat *const Iyy, dFloat *const Izz);
NEWTON_API void  N236_NewtonBodyGetInvMass(const N236_NewtonBody *const body, dFloat *const invMass, dFloat *const invIxx, dFloat *const invIyy, dFloat *const invIzz);
NEWTON_API void  N236_NewtonBodyGetOmega(const N236_NewtonBody *const body, dFloat *const vector);
NEWTON_API void  N236_NewtonBodyGetVelocity(const N236_NewtonBody *const body, dFloat *const vector);
NEWTON_API void  N236_NewtonBodyGetForce(const N236_NewtonBody *const body, dFloat *const vector);
NEWTON_API void  N236_NewtonBodyGetTorque(const N236_NewtonBody *const body, dFloat *const vector);
NEWTON_API void  N236_NewtonBodyGetForceAcc(const N236_NewtonBody *const body, dFloat *const vector);
NEWTON_API void  N236_NewtonBodyGetTorqueAcc(const N236_NewtonBody *const body, dFloat *const vector);
NEWTON_API void  N236_NewtonBodyGetCentreOfMass(const N236_NewtonBody *const body, dFloat *const com);

NEWTON_API dFloat N236_NewtonBodyGetLinearDamping(const N236_NewtonBody *const body);
NEWTON_API void  N236_NewtonBodyGetAngularDamping(const N236_NewtonBody *const body, dFloat *const vector);
NEWTON_API void  N236_NewtonBodyGetAABB(const N236_NewtonBody *const body, dFloat *const p0, dFloat *const p1);
NEWTON_API N236_NewtonJoint *N236_NewtonBodyGetFirstJoint(const N236_NewtonBody *const body);
NEWTON_API N236_NewtonJoint *N236_NewtonBodyGetNextJoint(const N236_NewtonBody *const body, const N236_NewtonJoint *const joint);
NEWTON_API N236_NewtonJoint *N236_NewtonBodyGetFirstContactJoint(const N236_NewtonBody *const body);
NEWTON_API N236_NewtonJoint *N236_NewtonBodyGetNextContactJoint(const N236_NewtonBody *const body, const N236_NewtonJoint *const contactJoint);

NEWTON_API void *N236_NewtonContactJointGetFirstContact(const N236_NewtonJoint *const contactJoint);
NEWTON_API void *N236_NewtonContactJointGetNextContact(const N236_NewtonJoint *const contactJoint, void *const contact);

NEWTON_API int N236_NewtonContactJointGetContactCount(const N236_NewtonJoint *const contactJoint);
NEWTON_API void N236_NewtonContactJointRemoveContact(N236_NewtonJoint *contactJoint, void *const contact);

NEWTON_API N236_NewtonMaterial *N236_NewtonContactGetMaterial(void *const contact);


NEWTON_API void  N236_NewtonBodyAddBuoyancyForce(N236_NewtonBody *const body, dFloat fluidDensity,
        dFloat fluidLinearViscosity, dFloat fluidAngularViscosity,
        const dFloat *const gravityVector, N236_NewtonGetBuoyancyPlane buoyancyPlane, void *const context);

//	NEWTON_API void N236_NewtonBodyForEachPolygonDo (const N236_NewtonBody* const body, N236_NewtonCollisionIterator callback);
NEWTON_API void N236_NewtonBodyAddImpulse(N236_NewtonBody *body, const dFloat *const pointDeltaVeloc, const dFloat *const pointPosit);


NEWTON_API void N236_NewtonBodyApplyImpulseArray(N236_NewtonBody *body, int impuleCount, int strideInByte, const dFloat *const impulseArray, const dFloat *const pointArray);


// **********************************************************************************************
//
// Common joint functions
//
// **********************************************************************************************
NEWTON_API void *N236_NewtonJointGetUserData(const N236_NewtonJoint *const joint);
NEWTON_API void N236_NewtonJointSetUserData(N236_NewtonJoint *joint, void *const userData);

NEWTON_API N236_NewtonBody *N236_NewtonJointGetBody0(const N236_NewtonJoint *const joint);
NEWTON_API N236_NewtonBody *N236_NewtonJointGetBody1(const N236_NewtonJoint *const joint);

NEWTON_API void N236_NewtonJointGetInfo(const N236_NewtonJoint *const joint, N236_NewtonJointRecord *const info);
NEWTON_API int N236_NewtonJointGetCollisionState(const N236_NewtonJoint *const joint);
NEWTON_API void N236_NewtonJointSetCollisionState(N236_NewtonJoint *joint, int state);

NEWTON_API dFloat N236_NewtonJointGetStiffness(const N236_NewtonJoint *const joint);
NEWTON_API void N236_NewtonJointSetStiffness(N236_NewtonJoint *joint, dFloat state);

NEWTON_API void N236_NewtonDestroyJoint(N236_NewtonWorld *newtonWorld, N236_NewtonJoint *joint);
NEWTON_API void N236_NewtonJointSetDestructor(const N236_NewtonJoint *const joint, N236_NewtonConstraintDestructor destructor);




// **********************************************************************************************
//
// Ball and Socket joint functions
//
// **********************************************************************************************
NEWTON_API N236_NewtonJoint *N236_NewtonConstraintCreateBall(N236_NewtonWorld *newtonWorld, const dFloat *pivotPoint,
        N236_NewtonBody *childBody, N236_NewtonBody *parentBody);
NEWTON_API void N236_NewtonBallSetUserCallback(N236_NewtonJoint *ball, N236_NewtonBallCallBack callback);
NEWTON_API void N236_NewtonBallGetJointAngle(const N236_NewtonJoint *const ball, dFloat *angle);
NEWTON_API void N236_NewtonBallGetJointOmega(const N236_NewtonJoint *const ball, dFloat *omega);
NEWTON_API void N236_NewtonBallGetJointForce(const N236_NewtonJoint *const ball, dFloat *const force);
NEWTON_API void N236_NewtonBallSetConeLimits(N236_NewtonJoint *ball, const dFloat *pin, dFloat maxConeAngle, dFloat maxTwistAngle);

// **********************************************************************************************
//
// Hinge joint functions
//
// **********************************************************************************************
NEWTON_API N236_NewtonJoint *N236_NewtonConstraintCreateHinge(N236_NewtonWorld *newtonWorld,
        const dFloat *pivotPoint, const dFloat *pinDir,
        N236_NewtonBody *childBody, N236_NewtonBody *parentBody);

NEWTON_API void N236_NewtonHingeSetUserCallback(N236_NewtonJoint *hinge, N236_NewtonHingeCallBack callback);
NEWTON_API dFloat N236_NewtonHingeGetJointAngle(const N236_NewtonJoint *const hinge);
NEWTON_API dFloat N236_NewtonHingeGetJointOmega(const N236_NewtonJoint *const hinge);
NEWTON_API void N236_NewtonHingeGetJointForce(const N236_NewtonJoint *const hinge, dFloat *const force);
NEWTON_API dFloat N236_NewtonHingeCalculateStopAlpha(const N236_NewtonJoint *const hinge, const N236_NewtonHingeSliderUpdateDesc *const desc, dFloat angle);

// **********************************************************************************************
//
// Slider joint functions
//
// **********************************************************************************************
NEWTON_API N236_NewtonJoint *N236_NewtonConstraintCreateSlider(N236_NewtonWorld *newtonWorld,
        const dFloat *pivotPoint, const dFloat *pinDir,
        N236_NewtonBody *childBody, N236_NewtonBody *parentBody);
NEWTON_API void N236_NewtonSliderSetUserCallback(N236_NewtonJoint *slider, N236_NewtonSliderCallBack callback);
NEWTON_API dFloat N236_NewtonSliderGetJointPosit(const N236_NewtonJoint *slider);
NEWTON_API dFloat N236_NewtonSliderGetJointVeloc(const N236_NewtonJoint *slider);
NEWTON_API void N236_NewtonSliderGetJointForce(const N236_NewtonJoint *const slider, dFloat *const force);
NEWTON_API dFloat N236_NewtonSliderCalculateStopAccel(const N236_NewtonJoint *const slider, const N236_NewtonHingeSliderUpdateDesc *const desc, dFloat position);


// **********************************************************************************************
//
// Corkscrew joint functions
//
// **********************************************************************************************
NEWTON_API N236_NewtonJoint *N236_NewtonConstraintCreateCorkscrew(N236_NewtonWorld *newtonWorld,
        const dFloat *pivotPoint, const dFloat *pinDir,
        N236_NewtonBody *childBody, N236_NewtonBody *parentBody);
NEWTON_API void N236_NewtonCorkscrewSetUserCallback(N236_NewtonJoint *corkscrew, N236_NewtonCorkscrewCallBack callback);
NEWTON_API dFloat N236_NewtonCorkscrewGetJointPosit(const N236_NewtonJoint *const corkscrew);
NEWTON_API dFloat N236_NewtonCorkscrewGetJointAngle(const N236_NewtonJoint *const corkscrew);
NEWTON_API dFloat N236_NewtonCorkscrewGetJointVeloc(const N236_NewtonJoint *const corkscrew);
NEWTON_API dFloat N236_NewtonCorkscrewGetJointOmega(const N236_NewtonJoint *const corkscrew);
NEWTON_API void N236_NewtonCorkscrewGetJointForce(const N236_NewtonJoint *const corkscrew, dFloat *const force);
NEWTON_API dFloat N236_NewtonCorkscrewCalculateStopAlpha(const N236_NewtonJoint *const corkscrew, const N236_NewtonHingeSliderUpdateDesc *const desc, dFloat angle);
NEWTON_API dFloat N236_NewtonCorkscrewCalculateStopAccel(const N236_NewtonJoint *const corkscrew, const N236_NewtonHingeSliderUpdateDesc *const desc, dFloat position);


// **********************************************************************************************
//
// Universal joint functions
//
// **********************************************************************************************
NEWTON_API N236_NewtonJoint *N236_NewtonConstraintCreateUniversal(const N236_NewtonWorld *const newtonWorld,
        const dFloat *pivotPoint, const dFloat *pinDir0, const dFloat *pinDir1,
        const N236_NewtonBody *const childBody, const N236_NewtonBody *const parentBody);
NEWTON_API void N236_NewtonUniversalSetUserCallback(const N236_NewtonJoint *const universal, N236_NewtonUniversalCallBack callback);
NEWTON_API dFloat N236_NewtonUniversalGetJointAngle0(const N236_NewtonJoint *const universal);
NEWTON_API dFloat N236_NewtonUniversalGetJointAngle1(const N236_NewtonJoint *const universal);
NEWTON_API dFloat N236_NewtonUniversalGetJointOmega0(const N236_NewtonJoint *const universal);
NEWTON_API dFloat N236_NewtonUniversalGetJointOmega1(const N236_NewtonJoint *const universal);
NEWTON_API void N236_NewtonUniversalGetJointForce(const N236_NewtonJoint *const universal, dFloat *const force);
NEWTON_API dFloat N236_NewtonUniversalCalculateStopAlpha0(const N236_NewtonJoint *const universal, const N236_NewtonHingeSliderUpdateDesc *const desc, dFloat angle);
NEWTON_API dFloat N236_NewtonUniversalCalculateStopAlpha1(const N236_NewtonJoint *const universal, const N236_NewtonHingeSliderUpdateDesc *const desc, dFloat angle);


// **********************************************************************************************
//
// Up vector joint functions
//
// **********************************************************************************************
NEWTON_API N236_NewtonJoint *N236_NewtonConstraintCreateUpVector(const N236_NewtonWorld *const newtonWorld, const dFloat *pinDir, const N236_NewtonBody *const body);
NEWTON_API void N236_NewtonUpVectorGetPin(const N236_NewtonJoint *const upVector, dFloat *pin);
NEWTON_API void N236_NewtonUpVectorSetPin(const N236_NewtonJoint *const upVector, const dFloat *pin);


// **********************************************************************************************
//
// User defined bilateral Joint
//
// **********************************************************************************************
NEWTON_API N236_NewtonJoint *N236_NewtonConstraintCreateUserJoint(const N236_NewtonWorld *const newtonWorld, int maxDOF,
        N236_NewtonUserBilateralCallBack callback,
        N236_NewtonUserBilateralGetInfoCallBack getInfo,
        const N236_NewtonBody *const childBody, const N236_NewtonBody *const parentBody) ;
NEWTON_API void N236_NewtonUserJointSetFeedbackCollectorCallback(const N236_NewtonJoint *const joint, N236_NewtonUserBilateralCallBack getFeedback);
NEWTON_API void N236_NewtonUserJointAddLinearRow(const N236_NewtonJoint *const joint, const dFloat *const pivot0, const dFloat *const pivot1, const dFloat *const dir);
NEWTON_API void N236_NewtonUserJointAddAngularRow(const N236_NewtonJoint *const joint, dFloat relativeAngle, const dFloat *const dir);
NEWTON_API void N236_NewtonUserJointAddGeneralRow(const N236_NewtonJoint *const joint, const dFloat *const jacobian0, const dFloat *const jacobian1);
NEWTON_API void N236_NewtonUserJointSetRowMinimumFriction(const N236_NewtonJoint *const joint, dFloat friction);
NEWTON_API void N236_NewtonUserJointSetRowMaximumFriction(const N236_NewtonJoint *const joint, dFloat friction);
NEWTON_API void N236_NewtonUserJointSetRowAcceleration(const N236_NewtonJoint *const joint, dFloat acceleration);
NEWTON_API void N236_NewtonUserJointSetRowSpringDamperAcceleration(const N236_NewtonJoint *const joint, dFloat springK, dFloat springD);
NEWTON_API void N236_NewtonUserJointSetRowStiffness(const N236_NewtonJoint *const joint, dFloat stiffness);
NEWTON_API dFloat N236_NewtonUserJointGetRowForce(const N236_NewtonJoint *const joint, int row);


// **********************************************************************************************
//
// Mesh joint functions
//
// **********************************************************************************************
NEWTON_API N236_NewtonMesh *N236_NewtonMeshCreate(const N236_NewtonWorld *const newtonWorld);
NEWTON_API N236_NewtonMesh *N236_NewtonMeshCreateFromMesh(const N236_NewtonMesh *const mesh);
NEWTON_API N236_NewtonMesh *N236_NewtonMeshCreateFromCollision(const N236_NewtonCollision *const collision);
NEWTON_API N236_NewtonMesh *N236_NewtonMeshConvexHull(const N236_NewtonWorld *const newtonWorld, int count, const dFloat *const vertexCloud, int strideInBytes, dFloat tolerance);
NEWTON_API N236_NewtonMesh *N236_NewtonMeshCreatePlane(const N236_NewtonWorld *const newtonWorld, const dFloat *const locationMatrix, dFloat witdth, dFloat breadth, int material, const dFloat *const textureMatrix0, const dFloat *const textureMatrix1);

NEWTON_API void N236_NewtonMeshDestroy(const N236_NewtonMesh *const mesh);

NEWTON_API void N236_NewtonMesApplyTransform(const N236_NewtonMesh *const mesh, const dFloat *const matrix);
NEWTON_API void N236_NewtonMeshCalculateOOBB(const N236_NewtonMesh *const mesh, dFloat *const matrix, dFloat *const x, dFloat *const y, dFloat *const z);

NEWTON_API void N236_NewtonMeshCalculateVertexNormals(const N236_NewtonMesh *const mesh, dFloat angleInRadians);
NEWTON_API void N236_NewtonMeshApplySphericalMapping(const N236_NewtonMesh *const mesh, int material);
NEWTON_API void N236_NewtonMeshApplyBoxMapping(const N236_NewtonMesh *const mesh, int front, int side, int top);
NEWTON_API void N236_NewtonMeshApplyCylindricalMapping(const N236_NewtonMesh *const mesh, int cylinderMaterial, int capMaterial);

NEWTON_API int N236_NewtonMeshIsOpenMesh(const N236_NewtonMesh *const mesh);
NEWTON_API void N236_NewtonMeshFixTJoints(const N236_NewtonMesh *const mesh);


NEWTON_API void N236_NewtonMeshPolygonize(const N236_NewtonMesh *const mesh);
NEWTON_API void N236_NewtonMeshTriangulate(const N236_NewtonMesh *const mesh);
NEWTON_API N236_NewtonMesh *N236_NewtonMeshUnion(const N236_NewtonMesh *const mesh, const N236_NewtonMesh *const clipper, const dFloat *const clipperMatrix);
NEWTON_API N236_NewtonMesh *N236_NewtonMeshDifference(const N236_NewtonMesh *const mesh, const N236_NewtonMesh *const clipper, const dFloat *const clipperMatrix);
NEWTON_API N236_NewtonMesh *N236_NewtonMeshIntersection(const N236_NewtonMesh *const mesh, const N236_NewtonMesh *const clipper, const dFloat *const clipperMatrix);
NEWTON_API void N236_NewtonMeshClip(const N236_NewtonMesh *const mesh, const N236_NewtonMesh *const clipper, const dFloat *const clipperMatrix, N236_NewtonMesh **const topMesh, N236_NewtonMesh **const bottomMesh);
NEWTON_API void N236_NewtonMeshPlaneClip(const N236_NewtonMesh *const mesh, dFloat *const planeMatrix, const dFloat *const planeTextureMatrix, int planeMaterial, N236_NewtonMesh **const topMesh, N236_NewtonMesh **const bottomMesh);

NEWTON_API N236_NewtonMesh *N236_NewtonMeshApproximateConvexDecomposition(const N236_NewtonMesh *const mesh, dFloat maxConcavity, int maxCount);
NEWTON_API N236_NewtonMesh *N236_NewtonMeshTetrahedralization(const N236_NewtonMesh *const mesh, int internalMaterial, const dFloat *const textureMatrix);
NEWTON_API N236_NewtonMesh *N236_NewtonMeshVoronoiDecomposition(const N236_NewtonMesh *const mesh, int pointCount, int pointStrideInBytes, const dFloat *const pointCloud, int internalMaterial, const dFloat *const textureMatrix);


NEWTON_API void N236_NewtonRemoveUnusedVertices(const N236_NewtonMesh *const mesh, int32 *const vertexRemapTable);

NEWTON_API void N236_NewtonMeshBeginFace(const N236_NewtonMesh *const mesh);
NEWTON_API void N236_NewtonMeshAddFace(const N236_NewtonMesh *const mesh, int vertexCount, const dFloat *const vertex, int strideInBytes, int materialIndex);
NEWTON_API void N236_NewtonMeshEndFace(const N236_NewtonMesh *const mesh);

NEWTON_API void N236_NewtonMeshBuildFromVertexListIndexList(const N236_NewtonMesh *const mesh,
        int faceCount, const int32 *const faceIndexCount, const int32 *const faceMaterialIndex,
        const dFloat *const vertex, int vertexStrideInBytes, const int32 *const vertexIndex,
        const dFloat *const normal, int normalStrideInBytes, const int32 *const normalIndex,
        const dFloat *const uv0, int uv0StrideInBytes, const int32 *const uv0Index,
        const dFloat *const uv1, int uv1StrideInBytes, const int32 *const uv1Index);



NEWTON_API void N236_NewtonMeshGetVertexStreams(const N236_NewtonMesh *const mesh,
        int vertexStrideInByte, dFloat *const vertex,
        int normalStrideInByte, dFloat *const normal,
        int uvStrideInByte0, dFloat *const uv0,
        int uvStrideInByte1, dFloat *const uv1);
NEWTON_API void N236_NewtonMeshGetIndirectVertexStreams(const N236_NewtonMesh *const mesh,
        int vertexStrideInByte, dFloat *const vertex, int *const vertexIndices, int *const vertexCount,
        int normalStrideInByte, dFloat *const normal, int *const normalIndices, int *const normalCount,
        int uvStrideInByte0, dFloat *const uv0, int *const uvIndices0, int *const uvCount0,
        int uvStrideInByte1, dFloat *const uv1, int *const uvIndices1, int *const uvCount1);
NEWTON_API void *N236_NewtonMeshBeginHandle(const N236_NewtonMesh *const mesh);
NEWTON_API void N236_NewtonMeshEndHandle(const N236_NewtonMesh *const mesh, void *const handle);
NEWTON_API int N236_NewtonMeshFirstMaterial(const N236_NewtonMesh *const mesh, void *const handle);
NEWTON_API int N236_NewtonMeshNextMaterial(const N236_NewtonMesh *const mesh, void *const handle, int materialId);
NEWTON_API int N236_NewtonMeshMaterialGetMaterial(const N236_NewtonMesh *const mesh, void *const handle, int materialId);
NEWTON_API int N236_NewtonMeshMaterialGetIndexCount(const N236_NewtonMesh *const mesh, void *const handle, int materialId);
NEWTON_API void N236_NewtonMeshMaterialGetIndexStream(const N236_NewtonMesh *const mesh, void *const handle, int materialId, int32 *const index);
NEWTON_API void N236_NewtonMeshMaterialGetIndexStreamShort(const N236_NewtonMesh *const mesh, void *const handle, int materialId, short int *const index);

NEWTON_API N236_NewtonMesh *N236_NewtonMeshCreateFirstSingleSegment(const N236_NewtonMesh *const mesh);
NEWTON_API N236_NewtonMesh *N236_NewtonMeshCreateNextSingleSegment(const N236_NewtonMesh *const mesh, const N236_NewtonMesh *const segment);

NEWTON_API N236_NewtonMesh *N236_NewtonMeshCreateFirstLayer(const N236_NewtonMesh *const mesh);
NEWTON_API N236_NewtonMesh *N236_NewtonMeshCreateNextLayer(const N236_NewtonMesh *const mesh, const N236_NewtonMesh *const segment);


NEWTON_API int N236_NewtonMeshGetTotalFaceCount(const N236_NewtonMesh *const mesh);
NEWTON_API int N236_NewtonMeshGetTotalIndexCount(const N236_NewtonMesh *const mesh);
NEWTON_API void N236_NewtonMeshGetFaces(const N236_NewtonMesh *const mesh, int32 *const faceIndexCount, int32 *const faceMaterial, void **const faceIndices);


NEWTON_API int N236_NewtonMeshGetPointCount(const N236_NewtonMesh *const mesh);
NEWTON_API int N236_NewtonMeshGetPointStrideInByte(const N236_NewtonMesh *const mesh);
NEWTON_API dFloat64 *N236_NewtonMeshGetPointArray(const N236_NewtonMesh *const mesh);
NEWTON_API dFloat64 *N236_NewtonMeshGetNormalArray(const N236_NewtonMesh *const mesh);
NEWTON_API dFloat64 *N236_NewtonMeshGetUV0Array(const N236_NewtonMesh *const mesh);
NEWTON_API dFloat64 *N236_NewtonMeshGetUV1Array(const N236_NewtonMesh *const mesh);

NEWTON_API int N236_NewtonMeshGetVertexCount(const N236_NewtonMesh *const mesh);
NEWTON_API int N236_NewtonMeshGetVertexStrideInByte(const N236_NewtonMesh *const mesh);
NEWTON_API dFloat64 *N236_NewtonMeshGetVertexArray(const N236_NewtonMesh *const mesh);


NEWTON_API void *N236_NewtonMeshGetFirstVertex(const N236_NewtonMesh *const mesh);
NEWTON_API void *N236_NewtonMeshGetNextVertex(const N236_NewtonMesh *const mesh, void *const vertex);
NEWTON_API int N236_NewtonMeshGetVertexIndex(const N236_NewtonMesh *const mesh, void *const vertex);

NEWTON_API void *N236_NewtonMeshGetFirstPoint(const N236_NewtonMesh *const mesh);
NEWTON_API void *N236_NewtonMeshGetNextPoint(const N236_NewtonMesh *const mesh, void *const point);
NEWTON_API int N236_NewtonMeshGetPointIndex(const N236_NewtonMesh *const mesh, const void *const point);
NEWTON_API int N236_NewtonMeshGetVertexIndexFromPoint(const N236_NewtonMesh *const mesh, void *const point);


NEWTON_API void *N236_NewtonMeshGetFirstEdge(const N236_NewtonMesh *const mesh);
NEWTON_API void *N236_NewtonMeshGetNextEdge(const N236_NewtonMesh *const mesh, void *const edge);
NEWTON_API void N236_NewtonMeshGetEdgeIndices(const N236_NewtonMesh *const mesh, const void *const edge, int32 *const v0, int32 *const v1);
//NEWTON_API void N236_NewtonMeshGetEdgePointIndices (const N236_NewtonMesh* const mesh, const void* const edge, int* const v0, int* const v1);

NEWTON_API void *N236_NewtonMeshGetFirstFace(const N236_NewtonMesh *const mesh);
NEWTON_API void *N236_NewtonMeshGetNextFace(const N236_NewtonMesh *const mesh, void *const face);
NEWTON_API int N236_NewtonMeshIsFaceOpen(const N236_NewtonMesh *const mesh, const void *const face);
NEWTON_API int N236_NewtonMeshGetFaceMaterial(const N236_NewtonMesh *const mesh, const void *const face);
NEWTON_API int N236_NewtonMeshGetFaceIndexCount(const N236_NewtonMesh *const mesh, const void *const face);
NEWTON_API void N236_NewtonMeshGetFaceIndices(const N236_NewtonMesh *const mesh, const void *const face, int *const indices);
NEWTON_API void N236_NewtonMeshGetFacePointIndices(const N236_NewtonMesh *const mesh, const void *const face, int *const indices);


#ifdef __cplusplus
}
#endif

#endif
