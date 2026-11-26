// HPL1Engine: the Newton 1.53 API that HPL1Engine uses, implemented on the
// Newton Game Dynamics 2.36 core (core236/, ScummVM's copy of 2.36).
//
// This file is ours (zlib licence, like the core). The core's own API is
// renamed with an N236_ prefix so it can sit next to the 1.53 names that HPL1
// calls. Only the 1.53 functions HPL1Engine and tests/newton153_test.cpp use
// are provided; see README.md for how each one maps onto 2.36.

#include "../newton-1.53/Newton.h" // the 1.53 API HPL1 uses

#include "core236/NewtonStdAfx.h" // 2.36 core
#include "core236/Newton.h" // 2.36 API, N236_ names

#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace {

//-----------------------------------------------------------------------------
// Per-body and per-world records
//-----------------------------------------------------------------------------

// 1.53 callbacks take fewer arguments than 2.36's, so each body keeps its
// 1.53 callbacks and the application's user data here. The record is the
// 2.36 body's user data.
// 2.36 reads and writes matrices as its 16-byte aligned dgMatrix (SSE
// loads and stores), but 1.53 took plain float arrays and HPL1 passes
// whatever it has. Every matrix crossing the shim goes through one of these.
struct alignas(16) AlignedMatrix {
	dFloat v[16];
	AlignedMatrix() {}
	explicit AlignedMatrix(const dFloat *m) { memcpy(v, m, sizeof(v)); }
};

// For an optional 1.53 offset matrix (NULL means identity).
struct OptionalMatrix {
	AlignedMatrix m;
	const dFloat *ptr = nullptr;
	explicit OptionalMatrix(const dFloat *src) {
		if (src) {
			memcpy(m.v, src, sizeof(m.v));
			ptr = m.v;
		}
	}
};

struct BodyRecord {
	void *userData = nullptr;
	NewtonApplyForceAndTorque forceCallback = nullptr;
	NewtonSetTransform transformCallback = nullptr;
	// NaN guard: the body's matrix after the last update where it was finite.
	AlignedMatrix lastGoodMatrix;
	bool hasLastGoodMatrix = false;
	bool nonFiniteReported = false;
};

struct MaterialRecord {
	void *userData = nullptr;
	NewtonContactBegin begin = nullptr;
	NewtonContactProcess process = nullptr;
	NewtonContactEnd end = nullptr;
};

struct WorldRecord {
	std::map<std::pair<int, int>, MaterialRecord *> materials;
	std::set<N236_NewtonBody *> bodies;
};

N236_NewtonWorld *W(const NewtonWorld *w) { return (N236_NewtonWorld *)w; }
N236_NewtonBody *B(const NewtonBody *b) { return (N236_NewtonBody *)b; }
N236_NewtonJoint *J(const NewtonJoint *j) { return (N236_NewtonJoint *)j; }
N236_NewtonCollision *C(const NewtonCollision *c) { return (N236_NewtonCollision *)c; }
dgBody *Body(const NewtonBody *b) { return (dgBody *)b; }

BodyRecord *Record(const NewtonBody *b) {
	return (BodyRecord *)N236_NewtonBodyGetUserData(B(b));
}

WorldRecord *Record(const NewtonWorld *w) {
	return (WorldRecord *)N236_NewtonWorldGetUserData(W(w));
}

void BodyDestroyed(const N236_NewtonBody *const body) {
	WorldRecord *world = (WorldRecord *)N236_NewtonWorldGetUserData(N236_NewtonBodyGetWorld(body));
	if (world)
		world->bodies.erase((N236_NewtonBody *)body);
	delete (BodyRecord *)N236_NewtonBodyGetUserData(body);
	N236_NewtonBodySetUserData((N236_NewtonBody *)body, nullptr);
}

// 1.53 disabled a body that left the world box. 2.36 stops simulating it
// but keeps its velocity; clear that too, as 1.53 did.
void LeftWorld(const N236_NewtonBody *const body, int) {
	const dFloat zero[3] = {0.0f, 0.0f, 0.0f};
	N236_NewtonBodySetVelocity((N236_NewtonBody *)body, zero);
	N236_NewtonBodySetOmega((N236_NewtonBody *)body, zero);
}

bool Finite(const dFloat *v, int n) {
	for (int i = 0; i < n; i++)
		if (!std::isfinite(v[i]))
			return false;
	return true;
}

// HPL1Engine's NaN guard (as in the Newton 4 layer): a body whose matrix or
// velocity becomes NaN/inf goes back to its last finite matrix and stops,
// instead of poisoning the rest of the world.
void GuardNonFinite(WorldRecord *world) {
	for (N236_NewtonBody *body : world->bodies) {
		BodyRecord *rec = (BodyRecord *)N236_NewtonBodyGetUserData(body);
		if (!rec)
			continue;
		AlignedMatrix m;
		dFloat v[3], w[3];
		N236_NewtonBodyGetMatrix(body, m.v);
		N236_NewtonBodyGetVelocity(body, v);
		N236_NewtonBodyGetOmega(body, w);
		if (Finite(m.v, 16) && Finite(v, 3) && Finite(w, 3)) {
			rec->lastGoodMatrix = m;
			rec->hasLastGoodMatrix = true;
			continue;
		}
		if (!rec->nonFiniteReported) {
			fprintf(stderr, "Newton: a body's position or velocity became NaN/inf; restoring its last valid position and stopping it.\n");
			rec->nonFiniteReported = true;
		}
		const dFloat zero[3] = {0.0f, 0.0f, 0.0f};
		N236_NewtonBodySetVelocity(body, zero);
		N236_NewtonBodySetOmega(body, zero);
		if (rec->hasLastGoodMatrix)
			N236_NewtonBodySetMatrix(body, rec->lastGoodMatrix.v);
		((dgBody *)body)->HplRest();
	}
}

void ForceTrampoline(N236_NewtonBody *const body, dFloat, int32) {
	BodyRecord *rec = (BodyRecord *)N236_NewtonBodyGetUserData(body);
	if (rec && rec->forceCallback)
		rec->forceCallback((const NewtonBody *)body);
}

void TransformTrampoline(const N236_NewtonBody *const body, const dFloat *const matrix, int32) {
	BodyRecord *rec = (BodyRecord *)N236_NewtonBodyGetUserData(body);
	if (rec && rec->transformCallback)
		rec->transformCallback((const NewtonBody *)body, matrix);
}

//-----------------------------------------------------------------------------
// Contacts
//-----------------------------------------------------------------------------

// 1.53 called begin, then process for each contact, then end, for one pair
// before going on to the next, and HPL1's material keeps the pair's bodies in
// members between the three. 2.36 calls its overlap callback for every pair
// first and its contact callback later, so all three 1.53 callbacks are run
// from the contact callback. A pair that begin rejects loses its contacts
// there, before the solver sees them, which has the same effect as 1.53
// rejecting it at the overlap test.

// The contact pair being processed: 1.53's contact getters take only the
// material, while 2.36's also want a body to orient the normal and force.
N236_NewtonBody *g_contactBody0 = nullptr;

int OverlapTrampoline(const N236_NewtonMaterial *const, const N236_NewtonBody *const,
                      const N236_NewtonBody *const, int32) {
	return 1;
}

void ContactsTrampoline(const N236_NewtonJoint *const contactJoint, dFloat, int32) {
	void *first = N236_NewtonContactJointGetFirstContact(contactJoint);
	if (!first)
		return;
	N236_NewtonMaterial *firstMaterial = N236_NewtonContactGetMaterial(first);
	MaterialRecord *rec = (MaterialRecord *)N236_NewtonMaterialGetMaterialPairUserData(firstMaterial);
	if (!rec)
		return;

	N236_NewtonBody *body0 = N236_NewtonJointGetBody0(contactJoint);
	N236_NewtonBody *body1 = N236_NewtonJointGetBody1(contactJoint);
	N236_NewtonBody *previousBody0 = g_contactBody0;
	g_contactBody0 = body0;

	std::vector<void *> rejected;
	bool rejectAll = false;
	if (rec->begin && !rec->begin((const NewtonMaterial *)firstMaterial,
	                              (const NewtonBody *)body0, (const NewtonBody *)body1)) {
		rejectAll = true;
	} else {
		for (void *c = first; c; c = N236_NewtonContactJointGetNextContact(contactJoint, c)) {
			N236_NewtonMaterial *material = N236_NewtonContactGetMaterial(c);
			if (rec->process && !rec->process((const NewtonMaterial *)material, (const NewtonContact *)c))
				rejected.push_back(c);
		}
		if (rec->end)
			rec->end((const NewtonMaterial *)firstMaterial);
	}

	if (rejectAll) {
		rejected.clear();
		for (void *c = first; c; c = N236_NewtonContactJointGetNextContact(contactJoint, c))
			rejected.push_back(c);
	}
	for (void *c : rejected)
		N236_NewtonContactJointRemoveContact((N236_NewtonJoint *)contactJoint, c);

	g_contactBody0 = previousBody0;
}

struct PolygonIteration {
	const NewtonBody *body;
	NewtonCollisionIterator callback;
};

void PolygonTrampoline(void *const userData, int vertexCount, const dFloat *const faceArray, int faceId) {
	PolygonIteration *it = (PolygonIteration *)userData;
	it->callback(it->body, vertexCount, faceArray, faceId);
}

} // namespace

extern "C" {

//-----------------------------------------------------------------------------
// World
//-----------------------------------------------------------------------------

NewtonWorld *NewtonCreate(NewtonAllocMemory, NewtonFreeMemory) {
	// ScummVM's copy sets up its global allocator explicitly; once per
	// process, kept until exit.
	static bool globalsReady = false;
	if (!globalsReady) {
		N236_NewtonInitGlobals();
		globalsReady = true;
	}
	N236_NewtonWorld *world = N236_NewtonCreate();
	N236_NewtonWorldSetUserData(world, new WorldRecord);
	// 1.53 created its world box at +-100 m (HPL1 then sets the level's).
	dFloat p0[3] = {-100.0f, -100.0f, -100.0f};
	dFloat p1[3] = {100.0f, 100.0f, 100.0f};
	N236_NewtonSetWorldSize(world, p0, p1);
	N236_NewtonSetBodyLeaveWorldEvent(world, LeftWorld);
	return (NewtonWorld *)world;
}

void NewtonDestroy(const NewtonWorld *newtonWorld) {
	WorldRecord *rec = Record(newtonWorld);
	N236_NewtonDestroy(W(newtonWorld));
	if (rec) {
		for (auto &m : rec->materials)
			delete m.second;
		delete rec;
	}
}

void NewtonUpdate(const NewtonWorld *newtonWorld, dFloat timestep) {
	// Checked before the step (a NaN velocity set by the game) and after it
	// (one the solver produced), so the core never integrates or sorts a
	// non-finite body into its broadphase.
	GuardNonFinite(Record(newtonWorld));
	N236_NewtonUpdate(W(newtonWorld), timestep);
	GuardNonFinite(Record(newtonWorld));
}

void NewtonSetWorldSize(const NewtonWorld *newtonWorld, const dFloat *minPoint, const dFloat *maxPoint) {
	N236_NewtonSetWorldSize(W(newtonWorld), minPoint, maxPoint);
}

void NewtonSetSolverModel(const NewtonWorld *newtonWorld, int model) {
	N236_NewtonSetSolverModel(W(newtonWorld), model);
}

void NewtonSetFrictionModel(const NewtonWorld *newtonWorld, int model) {
	N236_NewtonSetFrictionModel(W(newtonWorld), model);
}

void NewtonWorldFreezeBody(const NewtonWorld *, const NewtonBody *body) {
	Body(body)->HplRest();
}

void NewtonWorldUnfreezeBody(const NewtonWorld *, const NewtonBody *body) {
	Body(body)->HplWake();
}

void NewtonWorldRayCast(const NewtonWorld *newtonWorld, const dFloat *p0, const dFloat *p1,
                        NewtonWorldRayFilterCallback filter, void *userData,
                        NewtonWorldRayPrefilterCallback prefilter) {
	N236_NewtonWorldRayCast(W(newtonWorld), p0, p1, (N236_NewtonWorldRayFilterCallback)filter, userData,
	                        (N236_NewtonWorldRayPrefilterCallback)prefilter);
}

//-----------------------------------------------------------------------------
// Collision shapes
//-----------------------------------------------------------------------------

NewtonCollision *NewtonCreateNull(const NewtonWorld *newtonWorld) {
	return (NewtonCollision *)N236_NewtonCreateNull(W(newtonWorld));
}

NewtonCollision *NewtonCreateBox(const NewtonWorld *newtonWorld, dFloat dx, dFloat dy, dFloat dz, const dFloat *offsetMatrix) {
	return (NewtonCollision *)N236_NewtonCreateBox(W(newtonWorld), dx, dy, dz, 0, OptionalMatrix(offsetMatrix).ptr);
}

NewtonCollision *NewtonCreateSphere(const NewtonWorld *newtonWorld, dFloat radiusX, dFloat radiusY, dFloat radiusZ, const dFloat *offsetMatrix) {
	return (NewtonCollision *)N236_NewtonCreateSphere(W(newtonWorld), radiusX, radiusY, radiusZ, 0, OptionalMatrix(offsetMatrix).ptr);
}

NewtonCollision *NewtonCreateCapsule(const NewtonWorld *newtonWorld, dFloat radius, dFloat height, const dFloat *offsetMatrix) {
	return (NewtonCollision *)N236_NewtonCreateCapsule(W(newtonWorld), radius, height, 0, OptionalMatrix(offsetMatrix).ptr);
}

NewtonCollision *NewtonCreateCylinder(const NewtonWorld *newtonWorld, dFloat radius, dFloat height, const dFloat *offsetMatrix) {
	return (NewtonCollision *)N236_NewtonCreateCylinder(W(newtonWorld), radius, height, 0, OptionalMatrix(offsetMatrix).ptr);
}

NewtonCollision *NewtonCreateCompoundCollision(const NewtonWorld *newtonWorld, int count,
                                               NewtonCollision *const collisionPrimitiveArray[]) {
	return (NewtonCollision *)N236_NewtonCreateCompoundCollision(W(newtonWorld), count,
	                                                             (N236_NewtonCollision *const *)collisionPrimitiveArray, 0);
}

NewtonCollision *NewtonCreateTreeCollision(const NewtonWorld *newtonWorld, NewtonTreeCollisionCallback) {
	// HPL1 passes no callback.
	return (NewtonCollision *)N236_NewtonCreateTreeCollision(W(newtonWorld), 0);
}

void NewtonTreeCollisionBeginBuild(const NewtonCollision *treeCollision) {
	N236_NewtonTreeCollisionBeginBuild(C(treeCollision));
}

void NewtonTreeCollisionAddFace(const NewtonCollision *treeCollision, int vertexCount, const dFloat *vertexPtr,
                                int strideInBytes, int faceAttribute) {
	N236_NewtonTreeCollisionAddFace(C(treeCollision), vertexCount, vertexPtr, strideInBytes, faceAttribute);
}

void NewtonTreeCollisionEndBuild(const NewtonCollision *treeCollision, int optimize) {
	N236_NewtonTreeCollisionEndBuild(C(treeCollision), optimize);
}

void NewtonReleaseCollision(const NewtonWorld *newtonWorld, const NewtonCollision *collision) {
	N236_NewtonReleaseCollision(W(newtonWorld), C(collision));
}

void NewtonConvexCollisionCalculateInertialMatrix(const NewtonCollision *convexCollision, dFloat *inertia, dFloat *origin) {
	N236_NewtonConvexCollisionCalculateInertialMatrix(C(convexCollision), inertia, origin);
}

int NewtonCollisionCollide(const NewtonWorld *newtonWorld, int maxSize,
                           const NewtonCollision *collsionA, const dFloat *matrixA,
                           const NewtonCollision *collsionB, const dFloat *matrixB,
                           dFloat *contacts, dFloat *normals, dFloat *penetration) {
	const int count = N236_NewtonCollisionCollide(W(newtonWorld), maxSize, C(collsionA), AlignedMatrix(matrixA).v, C(collsionB), AlignedMatrix(matrixB).v,
	                                              contacts, normals, penetration, 0);
	// 1.53's normals point from B to A, so pushing A along them separates
	// the shapes (HPL1's character relies on this). 2.36 gives that for a
	// level mesh B, but from A to B against convex and compound shapes.
	const dgCollision *const shapeB = (const dgCollision *)collsionB;
	if (!shapeB->IsType(dgCollision::dgCollisionMesh_RTTI) && !shapeB->IsType(dgCollision::dgCollisionScene_RTTI)) {
		for (int i = 0; i < count * 3; i++)
			normals[i] = -normals[i];
	}
	return count;
}

//-----------------------------------------------------------------------------
// Bodies
//-----------------------------------------------------------------------------

NewtonBody *NewtonCreateBody(const NewtonWorld *newtonWorld, const NewtonCollision *collision) {
	static const dFloat identityValues[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
	const AlignedMatrix identity(identityValues);
	N236_NewtonBody *body = N236_NewtonCreateBody(W(newtonWorld), C(collision), identity.v);
	N236_NewtonBodySetUserData(body, new BodyRecord);
	N236_NewtonBodySetDestructorCallback(body, BodyDestroyed);
	// 1.53: "The default and minimum values for the thresholds is 0.01 for
	// speed and 10 for frames count." 2.36's world thresholds already match
	// 0.01; the 10 quiet steps are counted by the HPL1Engine helpers.
	((dgBody *)body)->HplSetFreezeSteps(10);
	Record(newtonWorld)->bodies.insert(body);
	return (NewtonBody *)body;
}

void NewtonDestroyBody(const NewtonWorld *newtonWorld, const NewtonBody *body) {
	N236_NewtonDestroyBody(W(newtonWorld), B(body));
}

void NewtonBodySetUserData(const NewtonBody *body, void *userData) {
	Record(body)->userData = userData;
}

void *NewtonBodyGetUserData(const NewtonBody *body) {
	BodyRecord *rec = Record(body);
	return rec ? rec->userData : nullptr;
}

void NewtonBodySetForceAndTorqueCallback(const NewtonBody *body, NewtonApplyForceAndTorque callback) {
	Record(body)->forceCallback = callback;
	N236_NewtonBodySetForceAndTorqueCallback(B(body), callback ? ForceTrampoline : nullptr);
}

void NewtonBodySetTransformCallback(const NewtonBody *body, NewtonSetTransform callback) {
	Record(body)->transformCallback = callback;
	N236_NewtonBodySetTransformCallback(B(body), callback ? TransformTrampoline : nullptr);
}

void NewtonBodySetMaterialGroupID(const NewtonBody *body, int id) {
	N236_NewtonBodySetMaterialGroupID(B(body), id);
}

void NewtonBodySetContinuousCollisionMode(const NewtonBody *body, unsigned state) {
	N236_NewtonBodySetContinuousCollisionMode(B(body), state);
}

int NewtonBodyGetContinuousCollisionMode(const NewtonBody *body) {
	return N236_NewtonBodyGetContinuousCollisionMode(B(body));
}

void NewtonBodySetMatrix(const NewtonBody *body, const dFloat *matrix) {
	AlignedMatrix m(matrix);
	N236_NewtonBodySetMatrix(B(body), m.v);
}

void NewtonBodyGetMatrix(const NewtonBody *body, dFloat *matrix) {
	AlignedMatrix m;
	N236_NewtonBodyGetMatrix(B(body), m.v);
	memcpy(matrix, m.v, sizeof(m.v));
}

void NewtonBodySetMassMatrix(const NewtonBody *body, dFloat mass, dFloat Ixx, dFloat Iyy, dFloat Izz) {
	N236_NewtonBodySetMassMatrix(B(body), mass, Ixx, Iyy, Izz);
}

void NewtonBodyGetMassMatrix(const NewtonBody *body, dFloat *mass, dFloat *Ixx, dFloat *Iyy, dFloat *Izz) {
	N236_NewtonBodyGetMassMatrix(B(body), mass, Ixx, Iyy, Izz);
}

void NewtonBodySetCentreOfMass(const NewtonBody *body, const dFloat *com) {
	N236_NewtonBodySetCentreOfMass(B(body), com);
}

void NewtonBodyGetCentreOfMass(const NewtonBody *body, dFloat *com) {
	N236_NewtonBodyGetCentreOfMass(B(body), com);
}

// 1.53 bodies given a velocity start moving; 2.36 leaves a resting body
// asleep until something touches it. Wake it (only if resting, so a body
// whose speed HPL1 clamps every frame still counts its quiet steps).
static void WakeIfMoving(const NewtonBody *body, const dFloat *v) {
	if ((v[0] != 0.0f || v[1] != 0.0f || v[2] != 0.0f) && !Body(body)->HplIsActive())
		Body(body)->HplWake();
}

void NewtonBodySetVelocity(const NewtonBody *body, const dFloat *velocity) {
	WakeIfMoving(body, velocity);
	N236_NewtonBodySetVelocity(B(body), velocity);
}

void NewtonBodyGetVelocity(const NewtonBody *body, dFloat *vector) {
	N236_NewtonBodyGetVelocity(B(body), vector);
}

void NewtonBodySetOmega(const NewtonBody *body, const dFloat *omega) {
	WakeIfMoving(body, omega);
	N236_NewtonBodySetOmega(B(body), omega);
}

void NewtonBodyGetOmega(const NewtonBody *body, dFloat *vector) {
	N236_NewtonBodyGetOmega(B(body), vector);
}

void NewtonBodySetForce(const NewtonBody *body, const dFloat *force) {
	N236_NewtonBodySetForce(B(body), force);
}

void NewtonBodyAddForce(const NewtonBody *body, const dFloat *force) {
	N236_NewtonBodyAddForce(B(body), force);
}

void NewtonBodyGetForce(const NewtonBody *body, dFloat *vector) {
	N236_NewtonBodyGetForce(B(body), vector);
}

void NewtonBodySetTorque(const NewtonBody *body, const dFloat *torque) {
	N236_NewtonBodySetTorque(B(body), torque);
}

void NewtonBodyAddTorque(const NewtonBody *body, const dFloat *torque) {
	N236_NewtonBodyAddTorque(B(body), torque);
}

void NewtonBodySetLinearDamping(const NewtonBody *body, dFloat linearDamp) {
	N236_NewtonBodySetLinearDamping(B(body), linearDamp);
}

dFloat NewtonBodyGetLinearDamping(const NewtonBody *body) {
	return N236_NewtonBodyGetLinearDamping(B(body));
}

void NewtonBodySetAngularDamping(const NewtonBody *body, const dFloat *angularDamp) {
	N236_NewtonBodySetAngularDamping(B(body), angularDamp);
}

void NewtonBodyGetAngularDamping(const NewtonBody *body, dFloat *vector) {
	N236_NewtonBodyGetAngularDamping(B(body), vector);
}

void NewtonBodySetAutoFreeze(const NewtonBody *body, int state) {
	N236_NewtonBodySetAutoSleep(B(body), state);
}

int NewtonBodyGetAutoFreeze(const NewtonBody *body) {
	return N236_NewtonBodyGetAutoSleep(B(body));
}

// 1.53's per-body speed limits are not used: 2.36 has only world limits, and
// they are what 1.53 derived from HPL1's usual per-body value (0.01). The
// step count is kept (see dgBody::HplSetFreezeSteps).
void NewtonBodySetFreezeTreshold(const NewtonBody *body, dFloat, dFloat, int framesCount) {
	Body(body)->HplSetFreezeSteps(framesCount);
}

int NewtonBodyGetSleepingState(const NewtonBody *body) {
	return Body(body)->HplIsActive();
}

// 1.53: change the velocity of the given point by pointDeltaVeloc. 2.36's
// NewtonBodyAddImpulse does the same.
void NewtonAddBodyImpulse(const NewtonBody *body, const dFloat *pointDeltaVeloc, const dFloat *pointPosit) {
	Body(body)->HplWake();
	N236_NewtonBodyAddImpulse(B(body), pointDeltaVeloc, pointPosit);
}

void NewtonBodyAddBuoyancyForce(const NewtonBody *body, dFloat fluidDensity,
                                dFloat fluidLinearViscosity, dFloat fluidAngularViscosity,
                                const dFloat *gravityVector, NewtonGetBuoyancyPlane buoyancyPlane, void *context) {
	N236_NewtonBodyAddBuoyancyForce(B(body), fluidDensity, fluidLinearViscosity, fluidAngularViscosity,
	                                gravityVector, (N236_NewtonGetBuoyancyPlane)buoyancyPlane, context);
}

void NewtonBodyForEachPolygonDo(const NewtonBody *body, NewtonCollisionIterator callback) {
	AlignedMatrix matrix;
	N236_NewtonBodyGetMatrix(B(body), matrix.v);
	PolygonIteration it = {body, callback};
	N236_NewtonCollisionForEachPolygonDo(N236_NewtonBodyGetCollision(B(body)), matrix.v, PolygonTrampoline, &it);
}

//-----------------------------------------------------------------------------
// Materials
//-----------------------------------------------------------------------------

int NewtonMaterialGetDefaultGroupID(const NewtonWorld *newtonWorld) {
	return N236_NewtonMaterialGetDefaultGroupID(W(newtonWorld));
}

int NewtonMaterialCreateGroupID(const NewtonWorld *newtonWorld) {
	return N236_NewtonMaterialCreateGroupID(W(newtonWorld));
}

void NewtonMaterialSetDefaultFriction(const NewtonWorld *newtonWorld, int id0, int id1, dFloat staticFriction, dFloat kineticFriction) {
	N236_NewtonMaterialSetDefaultFriction(W(newtonWorld), id0, id1, staticFriction, kineticFriction);
}

void NewtonMaterialSetDefaultElasticity(const NewtonWorld *newtonWorld, int id0, int id1, dFloat elasticCoef) {
	N236_NewtonMaterialSetDefaultElasticity(W(newtonWorld), id0, id1, elasticCoef);
}

void NewtonMaterialSetContinuousCollisionMode(const NewtonWorld *newtonWorld, int id0, int id1, int state) {
	N236_NewtonMaterialSetContinuousCollisionMode(W(newtonWorld), id0, id1, state);
}

void NewtonMaterialSetCollisionCallback(const NewtonWorld *newtonWorld, int id0, int id1, void *userData,
                                        NewtonContactBegin begin, NewtonContactProcess process, NewtonContactEnd end) {
	WorldRecord *world = Record(newtonWorld);
	std::pair<int, int> key(id0 < id1 ? id0 : id1, id0 < id1 ? id1 : id0);
	MaterialRecord *&rec = world->materials[key];
	if (!rec)
		rec = new MaterialRecord;
	rec->userData = userData;
	rec->begin = begin;
	rec->process = process;
	rec->end = end;
	const bool any = begin || process || end;
	N236_NewtonMaterialSetCollisionCallback(W(newtonWorld), id0, id1, rec,
	                                        any ? OverlapTrampoline : nullptr, any ? ContactsTrampoline : nullptr);
}

dFloat NewtonMaterialGetContactNormalSpeed(const NewtonMaterial *material, const NewtonContact *) {
	return N236_NewtonMaterialGetContactNormalSpeed((const N236_NewtonMaterial *)material);
}

dFloat NewtonMaterialGetContactTangentSpeed(const NewtonMaterial *material, const NewtonContact *, int index) {
	return N236_NewtonMaterialGetContactTangentSpeed((const N236_NewtonMaterial *)material, index);
}

void NewtonMaterialGetContactForce(const NewtonMaterial *material, dFloat *force) {
	N236_NewtonMaterialGetContactForce((const N236_NewtonMaterial *)material, g_contactBody0, force);
}

void NewtonMaterialGetContactPositionAndNormal(const NewtonMaterial *material, dFloat *posit, dFloat *normal) {
	N236_NewtonMaterialGetContactPositionAndNormal((const N236_NewtonMaterial *)material, g_contactBody0, posit, normal);
}

//-----------------------------------------------------------------------------
// Joints
//-----------------------------------------------------------------------------

void NewtonDestroyJoint(const NewtonWorld *newtonWorld, const NewtonJoint *joint) {
	N236_NewtonDestroyJoint(W(newtonWorld), J(joint));
}

void NewtonJointSetUserData(const NewtonJoint *joint, void *userData) {
	N236_NewtonJointSetUserData(J(joint), userData);
}

void *NewtonJointGetUserData(const NewtonJoint *joint) {
	return N236_NewtonJointGetUserData(J(joint));
}

void NewtonJointSetCollisionState(const NewtonJoint *joint, int state) {
	N236_NewtonJointSetCollisionState(J(joint), state);
}

int NewtonJointGetCollisionState(const NewtonJoint *joint) {
	return N236_NewtonJointGetCollisionState(J(joint));
}

void NewtonJointSetStiffness(const NewtonJoint *joint, dFloat state) {
	N236_NewtonJointSetStiffness(J(joint), state);
}

dFloat NewtonJointGetStiffness(const NewtonJoint *joint) {
	return N236_NewtonJointGetStiffness(J(joint));
}

void NewtonJointSetDestructor(const NewtonJoint *joint, NewtonConstraintDestructor destructor) {
	N236_NewtonJointSetDestructor(J(joint), (N236_NewtonConstraintDestructor)destructor);
}

NewtonJoint *NewtonConstraintCreateBall(const NewtonWorld *newtonWorld, const dFloat *pivotPoint,
                                        const NewtonBody *childBody, const NewtonBody *parentBody) {
	return (NewtonJoint *)N236_NewtonConstraintCreateBall(W(newtonWorld), pivotPoint, B(childBody), B(parentBody));
}

void NewtonBallSetConeLimits(const NewtonJoint *ball, const dFloat *pin, dFloat maxConeAngle, dFloat maxTwistAngle) {
	N236_NewtonBallSetConeLimits(J(ball), pin, maxConeAngle, maxTwistAngle);
}

void NewtonBallGetJointAngle(const NewtonJoint *ball, dFloat *angle) {
	N236_NewtonBallGetJointAngle(J(ball), angle);
}

void NewtonBallGetJointOmega(const NewtonJoint *ball, dFloat *omega) {
	N236_NewtonBallGetJointOmega(J(ball), omega);
}

void NewtonBallGetJointForce(const NewtonJoint *ball, dFloat *force) {
	N236_NewtonBallGetJointForce(J(ball), force);
}

NewtonJoint *NewtonConstraintCreateHinge(const NewtonWorld *newtonWorld, const dFloat *pivotPoint, const dFloat *pinDir,
                                         const NewtonBody *childBody, const NewtonBody *parentBody) {
	return (NewtonJoint *)N236_NewtonConstraintCreateHinge(W(newtonWorld), pivotPoint, pinDir, B(childBody), B(parentBody));
}

void NewtonHingeSetUserCallback(const NewtonJoint *hinge, NewtonHingeCallBack callback) {
	N236_NewtonHingeSetUserCallback(J(hinge), (N236_NewtonHingeCallBack)callback);
}

dFloat NewtonHingeGetJointAngle(const NewtonJoint *hinge) {
	return N236_NewtonHingeGetJointAngle(J(hinge));
}

dFloat NewtonHingeGetJointOmega(const NewtonJoint *hinge) {
	return N236_NewtonHingeGetJointOmega(J(hinge));
}

void NewtonHingeGetJointForce(const NewtonJoint *hinge, dFloat *force) {
	N236_NewtonHingeGetJointForce(J(hinge), force);
}

dFloat NewtonHingeCalculateStopAlpha(const NewtonJoint *hinge, const NewtonHingeSliderUpdateDesc *desc, dFloat angle) {
	return N236_NewtonHingeCalculateStopAlpha(J(hinge), (const N236_NewtonHingeSliderUpdateDesc *)desc, angle);
}

NewtonJoint *NewtonConstraintCreateSlider(const NewtonWorld *newtonWorld, const dFloat *pivotPoint, const dFloat *pinDir,
                                          const NewtonBody *childBody, const NewtonBody *parentBody) {
	return (NewtonJoint *)N236_NewtonConstraintCreateSlider(W(newtonWorld), pivotPoint, pinDir, B(childBody), B(parentBody));
}

void NewtonSliderSetUserCallback(const NewtonJoint *slider, NewtonSliderCallBack callback) {
	N236_NewtonSliderSetUserCallback(J(slider), (N236_NewtonSliderCallBack)callback);
}

dFloat NewtonSliderGetJointPosit(const NewtonJoint *slider) {
	return N236_NewtonSliderGetJointPosit(J(slider));
}

dFloat NewtonSliderGetJointVeloc(const NewtonJoint *slider) {
	return N236_NewtonSliderGetJointVeloc(J(slider));
}

void NewtonSliderGetJointForce(const NewtonJoint *slider, dFloat *force) {
	N236_NewtonSliderGetJointForce(J(slider), force);
}

dFloat NewtonSliderCalculateStopAccel(const NewtonJoint *slider, const NewtonHingeSliderUpdateDesc *desc, dFloat position) {
	return N236_NewtonSliderCalculateStopAccel(J(slider), (const N236_NewtonHingeSliderUpdateDesc *)desc, position);
}

NewtonJoint *NewtonConstraintCreateCorkscrew(const NewtonWorld *newtonWorld, const dFloat *pivotPoint, const dFloat *pinDir,
                                             const NewtonBody *childBody, const NewtonBody *parentBody) {
	return (NewtonJoint *)N236_NewtonConstraintCreateCorkscrew(W(newtonWorld), pivotPoint, pinDir, B(childBody), B(parentBody));
}

void NewtonCorkscrewSetUserCallback(const NewtonJoint *corkscrew, NewtonCorkscrewCallBack callback) {
	N236_NewtonCorkscrewSetUserCallback(J(corkscrew), (N236_NewtonCorkscrewCallBack)callback);
}

dFloat NewtonCorkscrewGetJointPosit(const NewtonJoint *corkscrew) {
	return N236_NewtonCorkscrewGetJointPosit(J(corkscrew));
}

dFloat NewtonCorkscrewGetJointAngle(const NewtonJoint *corkscrew) {
	return N236_NewtonCorkscrewGetJointAngle(J(corkscrew));
}

dFloat NewtonCorkscrewGetJointVeloc(const NewtonJoint *corkscrew) {
	return N236_NewtonCorkscrewGetJointVeloc(J(corkscrew));
}

void NewtonCorkscrewGetJointForce(const NewtonJoint *corkscrew, dFloat *force) {
	N236_NewtonCorkscrewGetJointForce(J(corkscrew), force);
}

dFloat NewtonCorkscrewCalculateStopAccel(const NewtonJoint *corkscrew, const NewtonHingeSliderUpdateDesc *desc, dFloat position) {
	return N236_NewtonCorkscrewCalculateStopAccel(J(corkscrew), (const N236_NewtonHingeSliderUpdateDesc *)desc, position);
}

} // extern "C"
