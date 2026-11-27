/* Headless test of the Newton 1.53 compatibility layer, as HPL1Engine uses it.
*
* Every check builds a small scene through the plain 1.53 API (Newton.h),
* steps it at 60 Hz and compares the numbers with what Newton 1.53 did (or
* Newton 2.36, its closest open relative, where 1.53's code is unavailable).
* Nothing is drawn, so it runs the same on any machine. Where it matters, the
* callbacks are copies of HPL1's own (impl/PhysicsJoint*Newton.cpp,
* impl/PhysicsMaterialNewton.cpp).
*
* Build and run (from the build folder):
*     make newton153_test && ./HPL1Engine/external/newton-1.53/newton153_test
* Exit code 0 = all checks passed. Each check prints PASS or FAIL.
*
* HPL1Engine addition; never sent upstream. zlib licence (see ../LICENSE).
*/

#include <Newton.h>   // deliberately first: it must compile on its own
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

//-----------------------------------------------------------------------------
// Helpers
//-----------------------------------------------------------------------------

static const float kStep = 1.0f / 60.0f;
static const float kIdentity[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
static int gFailures = 0;
static int gChecks = 0;

static void Section(const char* name) { printf("\n== %s\n", name); }
static void Check(const char* what, bool ok)
{
	gChecks++;
	printf("%s %s\n", ok ? "PASS" : "FAIL", what);
	if (!ok) gFailures++;
}
static void Translation(float* m, float x, float y, float z)
{
	memcpy(m, kIdentity, sizeof(kIdentity));
	m[12] = x; m[13] = y; m[14] = z;
}
static float Length(const float* v) { return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]); }
static void Step(NewtonWorld* w, int steps) { for (int i = 0; i < steps; i++) NewtonUpdate(w, kStep); }

static float gExtraUp = 0;   // extra upward force (N) some tests push with
static void Gravity(const NewtonBody* b)
{
	float m, ix, iy, iz;
	NewtonBodyGetMassMatrix(b, &m, &ix, &iy, &iz);
	float f[3] = {0, -9.81f * m + gExtraUp, 0};
	NewtonBodySetForce(b, f);
}
static void NoForce(const NewtonBody*) {}
static float gWaveAngle = 0;   // gravity tilt about z (rad), like the boat's sway
static void WaveGravity(const NewtonBody* b)
{
	float m, ix, iy, iz;
	NewtonBodyGetMassMatrix(b, &m, &ix, &iy, &iz);
	float f[3] = {std::sin(gWaveAngle) * 9.81f * m, -std::cos(gWaveAngle) * 9.81f * m, 0};
	NewtonBodySetForce(b, f);
}

// A box body. mass 0 = static. Dynamic boxes get gravity, no damping (the
// cores' defaults differ, and HPL1 sets damping on every entity body), and
// never auto-freeze unless the test says otherwise.
static NewtonBody* Box(NewtonWorld* w, float sx, float sy, float sz, float mass,
	float x, float y, float z, NewtonApplyForceAndTorque force = Gravity)
{
	NewtonCollision* c = NewtonCreateBox(w, sx, sy, sz, NULL);
	NewtonBody* b = NewtonCreateBody(w, c);
	NewtonReleaseCollision(w, c);
	float m[16]; Translation(m, x, y, z);
	NewtonBodySetMatrix(b, m);
	if (mass > 0)
	{
		const float s2 = (sx*sx + sy*sy + sz*sz) / 3.0f;
		NewtonBodySetMassMatrix(b, mass, mass*s2/6, mass*s2/6, mass*s2/6);
		NewtonBodySetForceAndTorqueCallback(b, force);
		NewtonBodySetAutoFreeze(b, 0);
		const float noDamping[3] = {0, 0, 0};
		NewtonBodySetLinearDamping(b, 0);
		NewtonBodySetAngularDamping(b, noDamping);
	}
	return b;
}
static float BodyY(const NewtonBody* b) { float m[16]; NewtonBodyGetMatrix(b, m); return m[13]; }
static float BodyX(const NewtonBody* b) { float m[16]; NewtonBodyGetMatrix(b, m); return m[12]; }
static float gTipY = 0;
static void TipMoved(const NewtonBody*, const float* m) { gTipY = m[13]; }

//-----------------------------------------------------------------------------
// 1. Buffers: functions that write into caller arrays stay inside them
//    (catches the 24-float matrix copy that overran 16-float buffers on GCC).
//-----------------------------------------------------------------------------

struct Guarded
{
	float pre[8]; float v[16]; float post[16];
	Guarded() { for (int i = 0; i < 8; i++) pre[i] = 777; for (int i = 0; i < 16; i++) { v[i] = 0; post[i] = 777; } }
	bool Intact() const
	{
		for (int i = 0; i < 8; i++) if (pre[i] != 777) return false;
		for (int i = 0; i < 16; i++) if (post[i] != 777) return false;
		return true;
	}
};

static void TestBuffers()
{
	Section("Buffers are not overrun");
	NewtonWorld* w = NewtonCreate(NULL, NULL);
	NewtonBody* b = Box(w, .5f, .5f, .5f, 1, 0, 0, 0);
	float piv[3] = {0, .5f, 0};
	NewtonJoint* j = NewtonConstraintCreateBall(w, piv, b, NULL);
	Step(w, 10);
	Guarded m, mass, ix, iy, iz, a, o, f;
	NewtonBodyGetMatrix(b, m.v);
	NewtonBodyGetMassMatrix(b, mass.v, ix.v, iy.v, iz.v);
	NewtonBallGetJointAngle(j, a.v);
	NewtonBallGetJointOmega(j, o.v);
	NewtonBallGetJointForce(j, f.v);
	Check("NewtonBodyGetMatrix writes 16 floats", m.Intact());
	Check("NewtonBodyGetMassMatrix", mass.Intact() && ix.Intact() && iy.Intact() && iz.Intact());
	Check("NewtonBallGetJointAngle/Omega/Force write 3 floats", a.Intact() && o.Intact() && f.Intact());
	NewtonDestroy(w);
}

//-----------------------------------------------------------------------------
// 2. NewtonCollisionCollide (HPL1's CheckShapeCollision: character, areas)
//    Shape A sits above B. 1.53: normals point from B to A, depth > 0.
//-----------------------------------------------------------------------------

static void CollidePair(NewtonWorld* w, const char* name, NewtonCollision* a, float ay, NewtonCollision* b, float depth)
{
	float ma[16], mb[16], p[96], n[96], d[32];
	Translation(ma, 0, ay, 0); Translation(mb, 0, 0, 0);
	int c = NewtonCollisionCollide(w, 32, a, ma, b, mb, p, n, d);
	bool ok = c > 0;
	for (int i = 0; i < c; i++) ok = ok && n[i*3+1] > 0.99f && std::fabs(d[i] - depth) < 0.02f;
	printf("  %s: %d points, normal (%.2f %.2f %.2f), depth %.3f (want %.3f)\n", name, c,
		c ? n[0] : 0, c ? n[1] : 0, c ? n[2] : 0, c ? d[0] : 0, depth);
	Check(name, ok);
	Translation(ma, 0, ay + 1.0f, 0);
	Check("  ...and no points when apart", NewtonCollisionCollide(w, 32, a, ma, b, mb, p, n, d) == 0);
}

static void TestCollisionCollide()
{
	Section("NewtonCollisionCollide");
	NewtonWorld* w = NewtonCreate(NULL, NULL);
	NewtonCollision* box2 = NewtonCreateBox(w, 2, 2, 2, NULL);   // top at y = 1
	NewtonCollision* box1 = NewtonCreateBox(w, 1, 1, 1, NULL);
	NewtonCollision* sphere = NewtonCreateSphere(w, 0.5f, 0.5f, 0.5f, NULL);
	CollidePair(w, "sphere into box", sphere, 1.3f, box2, 0.2f);
	CollidePair(w, "box into box", box1, 1.4f, box2, 0.1f);
	NewtonCollision* tree = NewtonCreateTreeCollision(w, NULL);
	NewtonTreeCollisionBeginBuild(tree);
	float quad[] = { -10,0,-10,  -10,0,10,  10,0,10,  10,0,-10 };
	NewtonTreeCollisionAddFace(tree, 4, quad, 3*sizeof(float), 0);
	NewtonTreeCollisionEndBuild(tree, 0);
	CollidePair(w, "sphere into a level floor (tree collision)", sphere, 0.35f, tree, 0.15f);
	NewtonReleaseCollision(w, box2); NewtonReleaseCollision(w, box1);
	NewtonReleaseCollision(w, sphere); NewtonReleaseCollision(w, tree);
	NewtonDestroy(w);
}

//-----------------------------------------------------------------------------
// 3. Joint readouts: force (HPL1 compares |F| with each joint's break force),
//    angle/position sign, ball angle/omega.
//-----------------------------------------------------------------------------

static void TestJointReadouts()
{
	Section("Joint force, angle and position readouts");
	NewtonWorld* w = NewtonCreate(NULL, NULL);
	NewtonSetSolverModel(w, 0);
	float pinZ[3] = {0,0,1}, pinX[3] = {1,0,0};
	NewtonBody* hb = Box(w, .5f, .5f, .5f, 1, 0, 5, 0);
	float hp[3] = {0, 5.5f, 0};
	NewtonJoint* hinge = NewtonConstraintCreateHinge(w, hp, pinZ, hb, NULL);
	NewtonBody* bb = Box(w, .5f, .5f, .5f, 1, 3, 5, 0);
	float bp[3] = {3, 5.5f, 0};
	NewtonJoint* ball = NewtonConstraintCreateBall(w, bp, bb, NULL);
	NewtonBody* sb = Box(w, .5f, .5f, .5f, 1, 6, 5, 0);
	float sp[3] = {6, 5, 0};
	NewtonJoint* slider = NewtonConstraintCreateSlider(w, sp, pinX, sb, NULL);   // horizontal rail
	Step(w, 300);
	float f[3];
	NewtonHingeGetJointForce(hinge, f);   printf("  hinge  |F| %.3f N\n", Length(f));
	Check("hinge holding 1 kg reports |F| ~ 9.81 N", std::fabs(Length(f) - 9.81f) < 0.5f);
	NewtonBallGetJointForce(ball, f);     printf("  ball   |F| %.3f N\n", Length(f));
	Check("ball holding 1 kg reports |F| ~ 9.81 N", std::fabs(Length(f) - 9.81f) < 0.5f);
	NewtonSliderGetJointForce(slider, f); printf("  slider |F| %.3f N\n", Length(f));
	Check("slider rail holding 1 kg reports |F| ~ 9.81 N", std::fabs(Length(f) - 9.81f) < 0.5f);
	float a[3], o[3];
	NewtonBallGetJointAngle(ball, a); NewtonBallGetJointOmega(ball, o);
	Check("ball at rest: angle and omega ~ 0", Length(a) < 0.05f && Length(o) < 0.05f);
	NewtonDestroy(w);

	// sign convention: +omega about the pin gives a growing +angle, +velocity
	// along the pin a growing +position (as 2.36)
	w = NewtonCreate(NULL, NULL);
	NewtonBody* h = Box(w, .5f, .5f, .5f, 1, 0, 0, 0, NoForce);
	float origin[3] = {0, 0, 0};
	NewtonJoint* hj = NewtonConstraintCreateHinge(w, origin, pinZ, h, NULL);
	NewtonBody* s = Box(w, .5f, .5f, .5f, 1, 5, 0, 0, NoForce);
	float sp2[3] = {5, 0, 0};
	NewtonJoint* sj = NewtonConstraintCreateSlider(w, sp2, pinX, s, NULL);
	float wz[3] = {0, 0, 1}, vx[3] = {0.5f, 0, 0};
	NewtonBodySetOmega(h, wz); NewtonBodySetVelocity(s, vx);
	Step(w, 30);
	printf("  after 0.5 s: hinge angle %.4f omega %.4f, slider posit %.4f veloc %.4f\n",
		NewtonHingeGetJointAngle(hj), NewtonHingeGetJointOmega(hj), NewtonSliderGetJointPosit(sj), NewtonSliderGetJointVeloc(sj));
	// 2.36 (like 1.x) reports the angle and position its joint callback saw
	// during the last step, i.e. before that step moved the body: one step
	// behind. Either reading is accepted; the point here is the sign.
	const float hingeAngle = NewtonHingeGetJointAngle(hj), sliderPosit = NewtonSliderGetJointPosit(sj);
	Check("hinge angle +0.5 rad and omega +1 after spinning +1 rad/s for 0.5 s",
		hingeAngle > 0.5f - 1.0f * kStep - 0.01f && hingeAngle < 0.51f && std::fabs(NewtonHingeGetJointOmega(hj) - 1) < 0.01f);
	Check("slider posit +0.25 and veloc +0.5 after 0.5 s at +0.5 m/s",
		sliderPosit > 0.25f - 0.5f * kStep - 0.01f && sliderPosit < 0.26f && std::fabs(NewtonSliderGetJointVeloc(sj) - 0.5f) < 0.01f);

	// the stop helpers (2.36 formulas): accel back to the stop + cancel the speed
	NewtonHingeSliderUpdateDesc d;
	d.m_timestep = kStep; d.m_accel = 0; d.m_minFriction = -1e20f; d.m_maxFriction = 1e20f;
	const float ang = NewtonHingeGetJointAngle(hj), om = NewtonHingeGetJointOmega(hj);
	const float pos = NewtonSliderGetJointPosit(sj), vel = NewtonSliderGetJointVeloc(sj);
	struct { const char* what; float got, want; } stops[] = {
		{"hinge stop alpha, past the upper stop", NewtonHingeCalculateStopAlpha(hj, &d, 0.3f), 100*(0.3f-ang) - om*1.01f/kStep},
		{"hinge stop alpha, below the lower stop (x16, speed ignored)", NewtonHingeCalculateStopAlpha(hj, &d, 0.7f), 100*16*(0.7f-ang)},
		{"hinge stop alpha, exactly at the stop", NewtonHingeCalculateStopAlpha(hj, &d, ang), 0},
		{"slider stop accel, past the upper stop", NewtonSliderCalculateStopAccel(sj, &d, 0.1f), 100*(0.1f-pos) - vel*1.01f/kStep},
		{"slider stop accel, below the lower stop", NewtonSliderCalculateStopAccel(sj, &d, 0.4f), 100*(0.4f-pos)},
	};
	for (unsigned i = 0; i < sizeof(stops)/sizeof(stops[0]); i++)
	{
		printf("  %s: %.3f (want %.3f)\n", stops[i].what, stops[i].got, stops[i].want);
		Check(stops[i].what, std::fabs(stops[i].got - stops[i].want) < 1e-2f * std::fmax(1.0f, std::fabs(stops[i].want)));
	}
	NewtonDestroy(w);
}

//-----------------------------------------------------------------------------
// 4. Bodies: sleeping state, freeze threshold, impulse, freeze/unfreeze,
//    continuous collision flag.
//-----------------------------------------------------------------------------

static void TestBodies()
{
	Section("Bodies: sleep, impulse, freeze");
	// Expected values below were measured with the original 1.53 library.
	NewtonWorld* w = NewtonCreate(NULL, NULL);
	NewtonMaterialSetDefaultFriction(w, 0, 0, 0, 0);   // frictionless floor: slow boxes slide forever
	Box(w, 100, 1, 100, 0, 0, -0.5f, 0);
	NewtonBody* plain = Box(w, .5f, .5f, .5f, 1, -5, 0.25f, 0);
	NewtonBody* slow = Box(w, .5f, .5f, .5f, 1, 5, 0.25f, 0);
	NewtonBody* slower = Box(w, .5f, .5f, .5f, 1, 10, 0.25f, 0);
	NewtonBodySetAutoFreeze(plain, 1); NewtonBodySetAutoFreeze(slow, 1); NewtonBodySetAutoFreeze(slower, 1);
	NewtonBodySetFreezeTreshold(slow, 0.01f, 0.01f, 10);   // HPL1's default
	NewtonBodySetFreezeTreshold(slower, 0.01f, 0.01f, 10);
	Step(w, 30);
	Check("a box settled on the floor rests: NewtonBodyGetSleepingState 0", NewtonBodyGetSleepingState(plain) == 0);
	float fast[3] = {1, 0, 0};
	const float plainX = BodyX(plain);
	NewtonBodySetVelocity(plain, fast);
	Step(w, 5);
	Check("NewtonBodySetVelocity doesn't wake a resting body (as 1.53)",
		NewtonBodyGetSleepingState(plain) == 0 && std::fabs(BodyX(plain) - plainX) < 1e-6f);
	NewtonWorldUnfreezeBody(w, plain); NewtonBodySetVelocity(plain, fast);
	Step(w, 5);
	Check("woken and given a velocity it moves: 1 = active", NewtonBodyGetSleepingState(plain) == 1 && BodyX(plain) - plainX > 0.05f);
	// 1.53's limits for a threshold of 0.01: speed^2 0.0022, accel^2 0.0264.
	// framesCount isn't used: a quiet body rests on the next step, and keeps
	// its velocity while resting.
	float v06[3] = {0.06f, 0, 0}, v03[3] = {0.03f, 0, 0};
	NewtonWorldUnfreezeBody(w, slow); NewtonBodySetVelocity(slow, v06);
	NewtonWorldUnfreezeBody(w, slower); NewtonBodySetVelocity(slower, v03);
	Step(w, 1);
	const float slowerX = BodyX(slower);
	Step(w, 20);
	float vs[3]; NewtonBodyGetVelocity(slower, vs);
	Check("0.06 m/s (speed^2 0.0036, over the limit) keeps sliding", NewtonBodyGetSleepingState(slow) == 1 && BodyX(slow) > 5.02f);
	Check("0.03 m/s (speed^2 0.0009) rests at once whatever framesCount says",
		NewtonBodyGetSleepingState(slower) == 0 && std::fabs(BodyX(slower) - slowerX) < 1e-6f);
	Check("...and keeps its velocity while resting", std::fabs(vs[0] - 0.03f) < 0.002f);
	NewtonDestroy(w);

	// Penumbra's boat (cEffect_WaveGravity) tilts gravity slowly and wakes
	// every body each frame. Waking alone doesn't move a quiet body in 1.53,
	// so a lamp hanging nearly still stays put until the tilt pushes it hard
	// enough: the 1.53 library stands it still on 44 of these 600 steps.
	w = NewtonCreate(NULL, NULL);
	NewtonBody* lamp = Box(w, .1f, .3f, .1f, 1.1f, 0, -0.6f, 0, WaveGravity);
	NewtonBodySetAutoFreeze(lamp, 1);
	NewtonBodySetFreezeTreshold(lamp, 0.01f, 0.01f, 10);
	float pivot[3] = {0, 0, 0};
	NewtonConstraintCreateBall(w, pivot, lamp, NULL);
	int stopped = 0;
	float lastX = BodyX(lamp);
	for (int i = 0; i < 600; i++)
	{
		gWaveAngle = 6.0f * 3.14159265f / 180.0f * std::sin(i * kStep * 2.0f * 3.14159265f / 8.0f);
		NewtonWorldUnfreezeBody(w, lamp);
		NewtonUpdate(w, kStep);
		if (std::fabs(BodyX(lamp) - lastX) < 1e-5f) stopped++;
		lastX = BodyX(lamp);
	}
	printf("  lamp in the boat's sway: stood still on %d of 600 steps (1.53: 44)\n", stopped);
	Check("a lamp woken every frame in the sway stands still about as often as in 1.53", stopped >= 30 && stopped <= 60);
	NewtonDestroy(w);

	// 1.53's NewtonBodySetMatrix keeps only the rotation: front normalised,
	// right = front x up, up = right x front. HPL1 passes scaled matrices
	// for map nodes that carry a scale (the boat's notebook).
	{
		w = NewtonCreate(NULL, NULL);
		NewtonBody* sb = Box(w, 1, 1, 1, 1, 0, 0, 0, NoForce);
		const float skewed[16] = {2, 0.5f, 0.1f, 0, 0.3f, 3, 0.2f, 0, 0.1f, 0.4f, 0.5f, 0, 1, 2, 3, 1};
		const float want[16] = {0.9690f, 0.2423f, 0.0485f, 0, -0.2447f, 0.9682f, 0.0529f, 0,
		                        -0.0341f, -0.0631f, 0.9974f, 0, 1, 2, 3, 1};   // from the 1.53 library
		NewtonBodySetMatrix(sb, skewed);
		float got[16]; NewtonBodyGetMatrix(sb, got);
		float err = 0;
		for (int i = 0; i < 16; i++) err = std::fmax(err, std::fabs(got[i] - want[i]));
		Check("NewtonBodySetMatrix drops scale and skew the way 1.53 did", err < 1e-3f);
		NewtonDestroy(w);
	}

	w = NewtonCreate(NULL, NULL);
	NewtonCollision* c = NewtonCreateBox(w, 1, 0.5f, 0.25f, NULL);
	NewtonBody* b = NewtonCreateBody(w, c);
	NewtonReleaseCollision(w, c);
	float m[16]; Translation(m, 1, 2, 3); NewtonBodySetMatrix(b, m);
	NewtonBodySetMassMatrix(b, 2, 0.1f, 0.2f, 0.3f);
	NewtonBodySetForceAndTorqueCallback(b, NoForce);
	// 1.53's impulse is "change the velocity of this point by this much"
	float dv[3] = {1, 0, 0}, centre[3] = {1, 2, 3};
	NewtonAddBodyImpulse(b, dv, centre);
	float vel[3], om[3];
	NewtonBodyGetVelocity(b, vel); NewtonBodyGetOmega(b, om);
	Check("impulse at the centre changes velocity by exactly dv, no spin",
		std::fabs(vel[0] - 1) < 1e-4f && Length(om) < 1e-5f);
	float v0[3], o0[3]; NewtonBodyGetVelocity(b, v0); NewtonBodyGetOmega(b, o0);
	float dv2[3] = {0.3f, -0.7f, 0.5f}, point[3] = {1.4f, 2.2f, 2.9f}, r[3] = {0.4f, 0.2f, -0.1f};
	NewtonAddBodyImpulse(b, dv2, point);
	NewtonBodyGetVelocity(b, vel); NewtonBodyGetOmega(b, om);
	float pointDv[3];
	for (int i = 0; i < 3; i++)
	{
		const int j = (i + 1) % 3, k = (i + 2) % 3;
		pointDv[i] = (vel[i] + om[j]*r[k] - om[k]*r[j]) - (v0[i] + o0[j]*r[k] - o0[k]*r[j]);
	}
	printf("  off-centre impulse: point velocity changed by (%.4f %.4f %.4f), wanted (0.3 -0.7 0.5)\n", pointDv[0], pointDv[1], pointDv[2]);
	Check("off-centre impulse changes that point's velocity by exactly dv, and spins the body",
		std::fabs(pointDv[0] - 0.3f) < 1e-3f && std::fabs(pointDv[1] + 0.7f) < 1e-3f && std::fabs(pointDv[2] - 0.5f) < 1e-3f && Length(om) > 1e-3f);
	Step(w, 1);
	NewtonWorldFreezeBody(w, b);
	Check("NewtonWorldFreezeBody: reports sleeping", NewtonBodyGetSleepingState(b) == 0);
	const float x0 = BodyX(b);
	Step(w, 10);
	Check("...and stays put", std::fabs(BodyX(b) - x0) < 1e-6f);
	float frozenV[3]; NewtonBodyGetVelocity(b, frozenV);
	Check("...keeping its velocity, as 1.53 did", Length(frozenV) > 0.5f);
	NewtonWorldUnfreezeBody(w, b);
	NewtonAddBodyImpulse(b, dv, centre);
	Step(w, 1);
	Check("unfreeze + impulse: active and moving again", NewtonBodyGetSleepingState(b) == 1 && std::fabs(BodyX(b) - x0) > 1e-3f);
	int a0 = NewtonBodyGetContinuousCollisionMode(b);
	NewtonBodySetContinuousCollisionMode(b, 1); int a1 = NewtonBodyGetContinuousCollisionMode(b);
	NewtonBodySetContinuousCollisionMode(b, 0); int a2 = NewtonBodyGetContinuousCollisionMode(b);
	Check("continuous collision flag reads back (0, 1, 0)", a0 == 0 && a1 == 1 && a2 == 0);
	NewtonDestroy(w);

	// A body put to rest after every step (HPL1's LimitAutoSleep does this to
	// a joint sitting at its limit) while the body it is hinged to swings:
	// as in the 1.53 library it wakes each update, moves with it and gets its
	// transform callback. Newton 4 alone dragged it along but never reported
	// its matrix (the outside hatch's wheel stayed behind on screen).
	w = NewtonCreate(NULL, NULL);
	NewtonBody* arm = Box(w, 1, 0.1f, 0.1f, 1, 0.5f, 0, 0);
	float armPin[3] = {0, 0, 1}, armPivot[3] = {0, 0, 0};
	NewtonConstraintCreateHinge(w, armPivot, armPin, arm, NULL);
	NewtonBody* tip = Box(w, 0.1f, 0.1f, 0.1f, 1, 1.0f, 0, 0, NoForce);
	float tipPivot[3] = {1.0f, 0, 0};
	NewtonConstraintCreateHinge(w, tipPivot, armPin, tip, arm);
	NewtonBodySetTransformCallback(tip, TipMoved);
	gTipY = 0;
	for (int i = 0; i < 30; i++)
	{
		NewtonUpdate(w, kStep);
		NewtonWorldFreezeBody(w, tip);
	}
	printf("  tip y %.3f, last reported y %.3f\n", BodyY(tip), gTipY);
	Check("body rested after each step still follows the body it is hinged to", BodyY(tip) < -0.3f && std::fabs(gTipY - BodyY(tip)) < 0.02f);
	NewtonDestroy(w);
}

//-----------------------------------------------------------------------------
// 5. Ball-and-socket cone limits (HPL1 ragdolls and chains)
//-----------------------------------------------------------------------------

static float SpinInCone(float ox, float oy, float oz)
{
	NewtonWorld* w = NewtonCreate(NULL, NULL);
	NewtonBody* b = Box(w, .5f, .5f, .5f, 1, 0, 5, 0, NoForce);
	float piv[3] = {0, 5.5f, 0}, axis[3] = {1, 0, 0};
	NewtonJoint* j = NewtonConstraintCreateBall(w, piv, b, NULL);
	NewtonBallSetConeLimits(j, axis, 30 * 3.14159265f / 180, 10 * 3.14159265f / 180);
	float om[3] = {ox, oy, oz};
	NewtonBodySetOmega(b, om);
	float maxDeg = 0;
	for (int i = 0; i < 120; i++)
	{
		NewtonUpdate(w, kStep);
		float m[16]; NewtonBodyGetMatrix(b, m);
		const float cosA = std::fmin(1.0f, std::fmax(-1.0f, (m[0] + m[5] + m[10] - 1) / 2));
		maxDeg = std::fmax(maxDeg, std::acos(cosA) * 180 / 3.14159265f);
	}
	NewtonDestroy(w);
	return maxDeg;
}

static void TestConeLimits()
{
	Section("Ball cone limits (axis +X, cone 30 deg, twist 10 deg)");
	const float twist = SpinInCone(3, 0, 0), tilt = SpinInCone(0, 0, 3);
	printf("  spun about the axis: stops at %.1f deg; spun across it: stops at %.1f deg\n", twist, tilt);
	Check("twist about the cone axis stops near 10 deg", twist < 12.0f);
	Check("tilt of the cone axis stops near 30 deg", tilt > 25.0f && tilt < 33.0f);
}

//-----------------------------------------------------------------------------
// 6. Hinge/slider/corkscrew limit callbacks, as HPL1's LimitCallback
//-----------------------------------------------------------------------------

static float gMin, gMax, gFriction;
static int gCalls;
static unsigned HingeLimit(const NewtonJoint* j, NewtonHingeSliderUpdateDesc* d)
{
	gCalls++;
	const float a = NewtonHingeGetJointAngle(j);
	if (a > gMax) { d->m_accel = NewtonHingeCalculateStopAlpha(j, d, gMax); d->m_maxFriction = 0; return 1; }
	if (a < gMin) { d->m_accel = NewtonHingeCalculateStopAlpha(j, d, gMin); d->m_minFriction = 0; return 1; }
	return 0;
}
static unsigned HingeFriction(const NewtonJoint* j, NewtonHingeSliderUpdateDesc* d)
{
	d->m_accel = -NewtonHingeGetJointOmega(j) / d->m_timestep;
	d->m_minFriction = -gFriction; d->m_maxFriction = gFriction;
	return 1;
}
static unsigned SliderLimit(const NewtonJoint* j, NewtonHingeSliderUpdateDesc* d)
{
	const float p = NewtonSliderGetJointPosit(j);
	if (p < gMin) { d->m_accel = NewtonSliderCalculateStopAccel(j, d, gMin); d->m_minFriction = 0; return 1; }
	if (p > gMax) { d->m_accel = NewtonSliderCalculateStopAccel(j, d, gMax); d->m_maxFriction = 0; return 1; }
	return 0;
}
static unsigned ScrewLimit(const NewtonJoint* j, NewtonHingeSliderUpdateDesc* d)
{
	const float p = NewtonCorkscrewGetJointPosit(j);
	if (p < gMin) { d->m_accel = NewtonCorkscrewCalculateStopAccel(j, d, gMin); d->m_minFriction = 0; return 1; }
	if (p > gMax) { d->m_accel = NewtonCorkscrewCalculateStopAccel(j, d, gMax); d->m_maxFriction = 0; return 1; }
	return 0;
}

static void TestLimitCallbacks()
{
	Section("Joint limit callbacks (doors, levers, drawers, screws)");
	float pinZ[3] = {0,0,1}, pinY[3] = {0,1,0}, origin[3] = {0,0,0};
	// an arm out along +X on a Z hinge; gravity swings it down
	for (int limited = 0; limited < 2; limited++)
	{
		NewtonWorld* w = NewtonCreate(NULL, NULL); NewtonSetSolverModel(w, 0);
		NewtonBody* b = Box(w, .5f, .5f, .5f, 1, 1, 0, 0);
		NewtonJoint* h = NewtonConstraintCreateHinge(w, origin, pinZ, b, NULL);
		gMin = -0.5f; gMax = 0.5f; gCalls = 0;
		if (limited) NewtonHingeSetUserCallback(h, HingeLimit);
		float lo = 1e9f, hi = -1e9f;
		for (int i = 0; i < 300; i++) { NewtonUpdate(w, kStep); const float a = NewtonHingeGetJointAngle(h); lo = std::fmin(lo, a); hi = std::fmax(hi, a); }
		const float a = NewtonHingeGetJointAngle(h);
		printf("  hinge %s: angle range [%.3f %.3f], rests at %.3f, callback calls %d\n", limited ? "limited to +-0.5" : "free", lo, hi, a, gCalls);
		if (!limited) { Check("free hinge swings well past 0.5 rad", std::fmax(-lo, hi) > 1.0f); NewtonDestroy(w); continue; }
		Check("callback runs once per update", gCalls == 300);
		Check("limited hinge stays within +-0.5 rad (+0.1 overshoot at 60 Hz)", lo > -0.6f && hi < 0.6f);
		Check("...and rests on the limit", std::fabs(std::fabs(a) - 0.5f) < 0.03f);
		gExtraUp = 30;
		Step(w, 20);
		const float a2 = NewtonHingeGetJointAngle(h);
		gExtraUp = 0;
		printf("  pushed the other way: %.3f\n", a2);
		Check("one-sided stop: the hinge leaves it freely when pushed away", std::fabs(a2 - a) > 0.1f && std::fabs(a2) < 0.6f);
		NewtonDestroy(w);
	}
	// a bounded stop-motor row ("friction"): holds above the load, slips below
	for (int strong = 0; strong < 2; strong++)
	{
		NewtonWorld* w = NewtonCreate(NULL, NULL); NewtonSetSolverModel(w, 0);
		NewtonBody* b = Box(w, .5f, .5f, .5f, 1, 1, 0, 0);
		NewtonJoint* h = NewtonConstraintCreateHinge(w, origin, pinZ, b, NULL);
		gFriction = strong ? 20.0f : 4.0f;
		NewtonHingeSetUserCallback(h, HingeFriction);
		Step(w, 30);
		const float a = NewtonHingeGetJointAngle(h);
		printf("  friction row bounded at %.0f N*m vs 9.81 N*m of load: angle after 0.5 s %.3f\n", gFriction, a);
		if (strong) Check("strong friction row holds the arm", std::fabs(a) < 0.02f);
		else Check("weak friction row lets it turn, slower than free fall", std::fabs(a) > 0.1f && std::fabs(a) < 1.2f);
		NewtonDestroy(w);
	}
	// slider and corkscrew on a vertical pin, limits [-1, 1]
	for (int screw = 0; screw < 2; screw++)
	{
		NewtonWorld* w = NewtonCreate(NULL, NULL); NewtonSetSolverModel(w, 0);
		NewtonBody* b = Box(w, .5f, .5f, .5f, 1, 0, 0, 0);
		NewtonJoint* j = screw ? NewtonConstraintCreateCorkscrew(w, origin, pinY, b, NULL) : NewtonConstraintCreateSlider(w, origin, pinY, b, NULL);
		gMin = -1; gMax = 1;
		if (screw) NewtonCorkscrewSetUserCallback(j, ScrewLimit); else NewtonSliderSetUserCallback(j, SliderLimit);
		float lo = 1e9f;
		for (int i = 0; i < 300; i++) { NewtonUpdate(w, kStep); lo = std::fmin(lo, screw ? NewtonCorkscrewGetJointPosit(j) : NewtonSliderGetJointPosit(j)); }
		const float p = screw ? NewtonCorkscrewGetJointPosit(j) : NewtonSliderGetJointPosit(j);
		gExtraUp = 30;
		Step(w, 60);
		gExtraUp = 0;
		const float up = screw ? NewtonCorkscrewGetJointPosit(j) : NewtonSliderGetJointPosit(j);
		printf("  %s: lowest %.3f, rests at %.3f; pushed up it stops at %.3f\n", screw ? "corkscrew" : "slider", lo, p, up);
		Check(screw ? "corkscrew stops at -1 and, pushed up, at +1" : "slider stops at -1 and, pushed up, at +1",
			lo > -1.05f && std::fabs(p + 1) < 0.02f && std::fabs(up - 1) < 0.05f);
		NewtonDestroy(w);
	}
}

//-----------------------------------------------------------------------------
// 7. Materials and contact callbacks, as HPL1's PhysicsMaterialNewton
//-----------------------------------------------------------------------------

static std::string gLog;
static std::thread::id gMainThread;
static bool gWrongThread, gInPair, gReject, gWatching;
static const NewtonBody* gWatch;
static int gWatchedBegins, gPoints;
static float gMaxNormalSpeed, gForceY, gLastForceY;

static int ContactBegin(const NewtonMaterial*, const NewtonBody* b0, const NewtonBody* b1)
{
	if (std::this_thread::get_id() != gMainThread) gWrongThread = true;
	gLog += 'B';
	if (gReject) return 0;
	gInPair = true;
	gWatching = (b0 == gWatch || b1 == gWatch);
	if (gWatching) gWatchedBegins++;
	gPoints = 0; gForceY = 0;
	return 1;
}
static int ContactProcess(const NewtonMaterial* m, const NewtonContact* c)
{
	gLog += 'P';
	if (gWatching)
	{
		gMaxNormalSpeed = std::fmax(gMaxNormalSpeed, NewtonMaterialGetContactNormalSpeed(m, c));
		float f[3]; NewtonMaterialGetContactForce(m, f);
		gForceY += std::fabs(f[1]); gPoints++;
	}
	return 1;
}
static void ContactEnd(const NewtonMaterial*)
{
	gLog += 'E';
	if (gWatching && gPoints) gLastForceY = gForceY;
	gInPair = false;
}

static void TestMaterials()
{
	Section("Materials and contact callbacks");
	gMainThread = std::this_thread::get_id();
	NewtonWorld* w = NewtonCreate(NULL, NULL); NewtonSetSolverModel(w, 0);
	const int def = NewtonMaterialGetDefaultGroupID(w);
	const int bouncy = NewtonMaterialCreateGroupID(w), dead = NewtonMaterialCreateGroupID(w), slick = NewtonMaterialCreateGroupID(w);
	// like HPL1's UpdateMaterials: every pair gets elasticity, friction and callbacks
	const int ids[4] = {def, bouncy, dead, slick};
	const float elasticity[4] = {0.5f, 1.0f, 0.0f, 0.5f}, friction[4] = {0.6f, 0.6f, 0.6f, 0.0f};
	for (int i = 0; i < 4; i++) for (int k = i; k < 4; k++)
	{
		const float e = (elasticity[i] == 0 || elasticity[k] == 0) ? 0 : (elasticity[i] + elasticity[k]) / 2;
		const float f = (friction[i] == 0 || friction[k] == 0) ? 0 : (friction[i] + friction[k]) / 2;
		NewtonMaterialSetDefaultElasticity(w, ids[i], ids[k], e);
		NewtonMaterialSetDefaultFriction(w, ids[i], ids[k], f, f);
		NewtonMaterialSetCollisionCallback(w, ids[i], ids[k], NULL, ContactBegin, ContactProcess, ContactEnd);
	}
	Box(w, 40, 1, 40, 0, 0, -0.5f, 0);   // floor, default group
	NewtonBody* bb = Box(w, .5f, .5f, .5f, 1, -4, 2, 0); NewtonBodySetMaterialGroupID(bb, bouncy);
	NewtonBody* db = Box(w, .5f, .5f, .5f, 1, 0, 2, 0);  NewtonBodySetMaterialGroupID(db, dead);
	NewtonBody* sb = Box(w, .5f, .5f, .5f, 1, 4, 0.25f, 0); NewtonBodySetMaterialGroupID(sb, slick);
	NewtonBody* rb = Box(w, .5f, .5f, .5f, 1, 4, 0.25f, 5);   // rough, default group
	float v[3] = {3, 0, 0}; NewtonBodySetVelocity(sb, v); NewtonBodySetVelocity(rb, v);
	gWatch = db;
	float bouncyTop = 0, deadTop = 0;
	int mostBegins = 0;
	for (int i = 0; i < 120; i++)
	{
		const int before = gWatchedBegins;
		NewtonUpdate(w, kStep);
		if (gWatchedBegins - before > mostBegins) mostBegins = gWatchedBegins - before;
		if (i > 40) { bouncyTop = std::fmax(bouncyTop, BodyY(bb)); deadTop = std::fmax(deadTop, BodyY(db)); }
	}
	// the log must read B, or B P...P E, over and over: no pair inside another
	bool ordered = true;
	for (size_t i = 0; i < gLog.size() && ordered; )
	{
		if (gLog[i] != 'B') { ordered = false; break; }
		size_t k = i + 1;
		while (k < gLog.size() && gLog[k] == 'P') k++;
		if (k > i + 1) { if (k >= gLog.size() || gLog[k] != 'E') ordered = false; k++; }
		i = k;
	}
	Check("callbacks run on the calling (game) thread", !gWrongThread);
	Check("each pair runs begin, process..., end before the next pair", ordered);
	Check("callbacks run once per pair per update", mostBegins == 1);
	printf("  dead box hit the floor at %.2f m/s (1.75 m drop: 5.86)\n", gMaxNormalSpeed);
	Check("contact normal speed is positive on impact, ~ the fall speed", gMaxNormalSpeed > 5.0f && gMaxNormalSpeed < 6.5f);
	printf("  contact force on the settling 1 kg box: %.2f N\n", gLastForceY);
	Check("NewtonMaterialGetContactForce sums to ~ m*g", std::fabs(gLastForceY - 9.81f) < 1.0f);
	printf("  after the first impact: bouncy box rose to %.2f m, dead box %.2f m\n", bouncyTop, deadTop);
	Check("per-pair elasticity: the bouncy box rebounds, the dead one doesn't", bouncyTop > 0.8f && deadTop < 0.3f);
	printf("  slid from 3 m/s: frictionless %.2f m, mu 0.6 %.2f m (expected 0.76)\n", BodyX(sb) - 4, BodyX(rb) - 4);
	Check("per-pair friction: frictionless slides on, mu 0.6 stops in ~0.76 m",
		BodyX(sb) - 4 > 5.5f && std::fabs(BodyX(rb) - 4 - 0.76f) < 0.15f);
	gReject = true;
	const float y0 = BodyY(db);
	Step(w, 30);
	gReject = false;
	printf("  dead box after begin returns 0: y %.2f -> %.2f\n", y0, BodyY(db));
	Check("a pair rejected by begin loses its contacts at once (falls through)", BodyY(db) < y0 - 1.0f);
	NewtonDestroy(w);
}

//-----------------------------------------------------------------------------
// 8. Joint utilities HPL1's joint wrapper uses
//-----------------------------------------------------------------------------

static int gDestroyed;
static void OnJointDestroyed(const NewtonJoint*) { gDestroyed++; }

static void TestJointUtilities()
{
	Section("Joint destroy, collision state, stiffness");
	NewtonWorld* w = NewtonCreate(NULL, NULL);
	NewtonBody* b = Box(w, 1, 1, 1, 1, 0, 5, 0);
	float piv[3] = {0, 5.5f, 0};
	NewtonJoint* j = NewtonConstraintCreateBall(w, piv, b, NULL);
	NewtonJointSetDestructor(j, OnJointDestroyed);
	Check("collision between jointed bodies is off by default", NewtonJointGetCollisionState(j) == 0);
	NewtonJointSetCollisionState(j, 1);
	Check("collision state can be switched on", NewtonJointGetCollisionState(j) == 1);
	NewtonJointSetCollisionState(j, 0);
	Check("stiffness defaults to 0.9 and reads back", std::fabs(NewtonJointGetStiffness(j) - 0.9f) < 1e-4f
		&& (NewtonJointSetStiffness(j, 0.5f), NewtonJointGetStiffness(j) == 0.5f));
	Step(w, 30);
	const float y0 = BodyY(b);
	gDestroyed = 0;
	NewtonDestroyJoint(w, j);
	Check("NewtonDestroyJoint runs the joint's destructor callback once", gDestroyed == 1);
	Step(w, 30);
	Check("...and the hanging box falls", BodyY(b) < y0 - 1.0f);
	// two overlapping boxes joined by a hinge: no push apart while collision is off
	NewtonBody* p = Box(w, 1, 1, 1, 0, 10, 0, 0);
	NewtonBody* c = Box(w, 1, 1, 1, 1, 10.5f, 0, 0);
	float hp[3] = {10.25f, 0, 0}, pin[3] = {0, 0, 1};
	NewtonConstraintCreateHinge(w, hp, pin, c, p);
	Step(w, 30);
	Check("overlapping jointed bodies aren't pushed apart", BodyX(c) - 10.5f < 0.01f);
	NewtonDestroy(w);
}

//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// 9. Safety: 1.53's world box, and bodies whose state becomes NaN.
//-----------------------------------------------------------------------------

static float gPushX = 0;
static void PushX(const NewtonBody* b)
{
	float m, ix, iy, iz;
	NewtonBodyGetMassMatrix(b, &m, &ix, &iy, &iz);
	float f[3] = {gPushX * m, 0, 0};
	NewtonBodySetForce(b, f);
}

// Penumbra's grab damping: torque = -omega * 0.9 * mass, every step.
static void DampSpin(const NewtonBody* b)
{
	float m, ix, iy, iz, om[3];
	NewtonBodyGetMassMatrix(b, &m, &ix, &iy, &iz);
	NewtonBodyGetOmega(b, om);
	float t[3] = {-om[0] * 0.9f * m, -om[1] * 0.9f * m, -om[2] * 0.9f * m};
	NewtonBodySetTorque(b, t);
}

static void TestSafety()
{
	Section("Safety: world box and NaN");
	NewtonWorld* w = NewtonCreate(NULL, NULL);
	float mn[3] = {-20, -20, -20}, mx[3] = {20, 20, 20};
	NewtonSetWorldSize(w, mn, mx);

	// A body pushed hard stops at the edge of the box (1.53: "automatically
	// disabled"), instead of flying on for ever.
	gPushX = 500;
	NewtonBody* flier = Box(w, 1, 1, 1, 1, 0, 0, 0, PushX);
	Step(w, 600);
	float v[3]; NewtonBodyGetVelocity(flier, v);
	Check("body leaving the world box stops near its edge", BodyX(flier) < 25 && BodyX(flier) > 19);
	Check("and is left with no velocity", Length(v) < 1e-3f);
	Step(w, 60);
	Check("and stays there", BodyX(flier) < 25);

	// A body inside the box is untouched (falls freely for 1 s: ~4.9 m).
	NewtonBody* faller = Box(w, 1, 1, 1, 1, -10, 10, 0);
	Step(w, 60);
	Check("body inside the box moves normally", BodyY(faller) < 6 && BodyY(faller) > 4.5f);

	// NaN velocity: the body goes back to its last valid position and stops.
	NewtonBody* bad = Box(w, 1, 1, 1, 1, 10, 0, 0, NoForce);
	Step(w, 2);
	float nanv[3] = {std::nanf(""), 0, 0};
	NewtonBodySetVelocity(bad, nanv);
	Step(w, 3);
	float m[16]; NewtonBodyGetMatrix(bad, m);
	NewtonBodyGetVelocity(bad, v);
	bool finite = true;
	for (int i = 0; i < 16; i++) if (!std::isfinite(m[i])) finite = false;
	Check("NaN velocity: position stays valid", finite && std::fabs(m[12] - 10) < 0.01f);
	Check("NaN velocity: body is stopped", std::isfinite(v[0]) && Length(v) < 1e-3f);
	gPushX = 0;
	NewtonDestroy(w);

	// 1.53/2.x never turn a body more than 45 degrees per step. A damping
	// torque far too strong for a thin body's inertia (Penumbra's grab on the
	// boat lamp) overshoots each step; without the cap the spin grows without
	// bound and the body is thrown away.
	w = NewtonCreate(NULL, NULL);
	NewtonBody* thin = Box(w, 0.3f, 0.02f, 0.02f, 1, 0, 0, 0, DampSpin);
	NewtonBodySetMassMatrix(thin, 1, 0.002f, 0.03f, 0.03f); // thin about x, like the lamp
	float om0[3] = {1, 0, 0};
	NewtonBodySetOmega(thin, om0);
	float om[3], maxSpin = 0;
	for (int i = 0; i < 120; i++)
	{
		NewtonUpdate(w, kStep);
		NewtonBodyGetOmega(thin, om);
		if (!(Length(om) <= maxSpin)) maxSpin = Length(om);
	}
	float pos[16]; NewtonBodyGetMatrix(thin, pos);
	Check("overshooting damping torque: spin capped at 45 degrees per step", maxSpin < 47.2f && std::isfinite(maxSpin));
	Check("and the body stays where it was", std::fabs(pos[12]) < 0.01f && std::fabs(pos[13]) < 0.01f);
	NewtonDestroy(w);
}

//-----------------------------------------------------------------------------
// 10. Level meshes (tree collision), as 1.53: bodies rest, stack and
//     slide on a floor made of many faces, a body that starts partly buried
//     in level geometry is not shoved sideways (the boat lamp's rod, whose
//     hook sits inside its ceiling beam), and overlapping bodies are pushed
//     apart as gently as the 1.53 library does.
//-----------------------------------------------------------------------------

// Adds a quad (corners in order, front side by the right-hand rule) as two
// triangles, as HPL1 feeds level meshes to Newton.
static void AddQuad(NewtonCollision* tree, const float* a, const float* b, const float* c, const float* d)
{
	float t0[9] = {a[0],a[1],a[2], b[0],b[1],b[2], c[0],c[1],c[2]};
	float t1[9] = {a[0],a[1],a[2], c[0],c[1],c[2], d[0],d[1],d[2]};
	NewtonTreeCollisionAddFace(tree, 3, t0, 3*sizeof(float), 0);
	NewtonTreeCollisionAddFace(tree, 3, t1, 3*sizeof(float), 0);
}

static NewtonBody* LevelBody(NewtonWorld* w, NewtonCollision* tree)
{
	NewtonBody* b = NewtonCreateBody(w, tree);
	NewtonBodySetMatrix(b, kIdentity);
	NewtonReleaseCollision(w, tree);
	return b;
}

// A floor at y = 0 from -4 to 4 in x and z, in 1 m tiles, plus a box beam
// (from y = 2.365 to 2.475, z from -0.0223 to 0.0523) like the boat's.
static void BuildLevel(NewtonWorld* w)
{
	NewtonCollision* tree = NewtonCreateTreeCollision(w, NULL);
	NewtonTreeCollisionBeginBuild(tree);
	for (int x = -4; x < 4; x++)
		for (int z = -4; z < 4; z++)
		{
			float a[3] = {(float)x, 0, (float)z}, b[3] = {(float)x, 0, (float)z + 1};
			float c[3] = {(float)x + 1, 0, (float)z + 1}, d[3] = {(float)x + 1, 0, (float)z};
			AddQuad(tree, a, b, c, d);
		}
	const float y0 = 2.365f, y1 = 2.475f, z0 = -0.0223f, z1 = 0.0523f, x0 = -2, x1 = 2;
	float p[8][3] = {{x0,y0,z0},{x1,y0,z0},{x1,y0,z1},{x0,y0,z1},{x0,y1,z0},{x1,y1,z0},{x1,y1,z1},{x0,y1,z1}};
	AddQuad(tree, p[0], p[1], p[2], p[3]);   // underside, facing down
	AddQuad(tree, p[4], p[7], p[6], p[5]);   // top, facing up
	AddQuad(tree, p[0], p[4], p[5], p[1]);   // side facing -z
	AddQuad(tree, p[3], p[2], p[6], p[7]);   // side facing +z
	NewtonTreeCollisionEndBuild(tree, 1);
	LevelBody(w, tree);
}

static void TestLevelMesh()
{
	Section("Level meshes");
	NewtonWorld* w = NewtonCreate(NULL, NULL);
	BuildLevel(w);

	// A box dropped on the tiled floor comes to rest on it.
	NewtonBody* box = Box(w, 0.5f, 0.5f, 0.5f, 1, 0.3f, 1, 0.3f);
	// Three boxes stacked across tile edges stay stacked.
	NewtonBody* s0 = Box(w, 0.5f, 0.5f, 0.5f, 1, -2, 0.25f, 1);
	NewtonBody* s1 = Box(w, 0.5f, 0.5f, 0.5f, 1, -2, 0.75f, 1);
	NewtonBody* s2 = Box(w, 0.5f, 0.5f, 0.5f, 1, -2, 1.25f, 1);
	// A box sliding over many tile edges keeps its height and slows down.
	NewtonBody* slider = Box(w, 0.5f, 0.5f, 0.5f, 1, 2, 0.25f, -3);
	float v[3] = {0, 0, 4};
	NewtonBodySetVelocity(slider, v);
	Step(w, 180);

	float m[16];
	NewtonBodyGetMatrix(box, m);
	Check("box dropped on a tiled floor rests on it", std::fabs(m[13] - 0.25f) < 0.02f);
	NewtonBodyGetMatrix(s2, m);
	Check("three stacked boxes stay stacked", std::fabs(m[13] - 1.25f) < 0.03f && std::fabs(m[12] + 2) < 0.05f && std::fabs(m[14] - 1) < 0.05f);
	(void)s0; (void)s1;
	NewtonBodyGetMatrix(slider, m);
	NewtonBodyGetVelocity(slider, v);
	Check("box sliding over tile edges keeps its height", std::fabs(m[13] - 0.25f) < 0.02f);
	Check("and slides on before friction stops it", m[14] > -2.5f && Length(v) < 0.5f);

	// The boat lamp's rod: a 0.85 m bar on a ball joint at y = 2.397, with
	// its top 6.6 cm up inside the beam. Old Newton leaves it hanging
	// straight; Newton 4 alone pushed it to its 30 degree cone limit.
	NewtonBody* rod = Box(w, 0.052168f, 0.847362f, 0.053070f, 1.1f, 0, 2.397f - 0.389f, 0);
	float pivot[3] = {0, 2.397f, 0}, pin[3] = {0, 1, 0};
	NewtonJoint* j = NewtonConstraintCreateBall(w, pivot, rod, NULL);
	NewtonBallSetConeLimits(j, pin, 30 * 3.14159265f / 180, 10 * 3.14159265f / 180);
	float maxTilt = 0;
	for (int i = 0; i < 120; i++)
	{
		NewtonUpdate(w, kStep);
		NewtonBodyGetMatrix(rod, m);
		maxTilt = std::fmax(maxTilt, std::acos(std::fmin(1.0f, m[5])) * 180 / 3.14159265f);
	}
	Check("bar hooked into a ceiling beam hangs straight (within 2 degrees)", maxTilt < 2);
	NewtonDestroy(w);

	// How hard a body that starts inside the floor is pushed out, with no
	// other forces. The 1.53 library: nothing up to 1/128 m of overlap, then
	// 1 m/s per metre beyond it (0.0122 m/s from 2 cm). Newton 4 alone gave
	// 0.033 m/s from 7.5 mm and 0.095 m/s from 2 cm, and kept that speed.
	float pushed[2];
	const float overlap[2] = {0.0075f, 0.02f};
	for (int k = 0; k < 2; k++)
	{
		w = NewtonCreate(NULL, NULL);
		BuildLevel(w);
		NewtonBody* sunk = Box(w, 0.1f, 0.05f, 0.1f, 1, 0.5f, 0.025f - overlap[k], 0.5f, NoForce);
		NewtonUpdate(w, kStep);
		NewtonBodyGetVelocity(sunk, v);
		pushed[k] = v[1];
		NewtonDestroy(w);
	}
	Check("body 7.5 mm inside the floor is not pushed out", std::fabs(pushed[0]) < 0.001f);
	Check("body 2 cm inside the floor is pushed out at about 0.012 m/s", pushed[1] > 0.009f && pushed[1] < 0.015f);

	// The rod tilted 10 degrees, so its top pokes out of the beam's -z side
	// and its centre is in front of that face. The shortest way off that
	// face is through it, so 1.53 makes no contact with it and the rod
	// stays still. Newton 4 alone pushed it out sideways at 0.04 m/s.
	w = NewtonCreate(NULL, NULL);
	BuildLevel(w);
	rod = Box(w, 0.052168f, 0.847362f, 0.053070f, 1.1f, 0, 0, 0, NoForce);
	const float tilt = 10 * 3.14159265f / 180;
	const float c = std::cos(tilt), s = std::sin(tilt);
	const float tilted[16] = {1,0,0,0, 0,c,s,0, 0,-s,c,0, 0,2.397f - 0.393f*c,-0.393f*s,1};
	NewtonBodySetMatrix(rod, tilted);
	NewtonUpdate(w, kStep);
	NewtonBodyGetVelocity(rod, v);
	Check("bar poking out of a beam's side is not pushed sideways", Length(v) < 0.002f);
	NewtonDestroy(w);
}

// Inertia of a shape offset from its body, as NewtonBodySetMassMatrix
// takes it: about the shape's centre of mass, so the same as the centred
// shape's (1.53). Newton 4 alone gave it about the body origin.
static void TestOffsetInertia()
{
	Section("Inertia of offset shapes");
	NewtonWorld* w = NewtonCreate(NULL, NULL);
	float offset[16];
	Translation(offset, -0.041f, 1.211f, -0.098f);
	NewtonCollision* moved = NewtonCreateBox(w, 0.37f, 0.412f, 0.224f, offset);
	NewtonCollision* centred = NewtonCreateBox(w, 0.37f, 0.412f, 0.224f, NULL);
	float i0[3], i1[3], c0[3], c1[3];
	NewtonConvexCollisionCalculateInertialMatrix(moved, i0, c0);
	NewtonConvexCollisionCalculateInertialMatrix(centred, i1, c1);
	bool same = true;
	for (int k = 0; k < 3; k++) same = same && (std::fabs(i0[k] - i1[k]) < 1e-4f);
	Check("offset box: inertia about its centre, as when centred", same);
	Check("...and its centre of mass is the offset", std::fabs(c0[1] - 1.211f) < 1e-4f && std::fabs(c0[0] + 0.041f) < 1e-4f);
	NewtonReleaseCollision(w, moved);
	NewtonReleaseCollision(w, centred);
	NewtonDestroy(w);
}

// A can fired at things. With continuous collision on for the body and the
// material pair (HPL1 turns both on), the 1.53 library makes it bounce back
// off a level wall, a 1 cm thin box and, with a two-part (compound) body, a
// wall again, at speeds where a single step carries it right through. With
// the body's switch off it passes through the wall, as in 1.53.
static float FireAt(int target, float speed, int continuous, bool compound)
{
	NewtonWorld* w = NewtonCreate(NULL, NULL);
	const int group = NewtonMaterialGetDefaultGroupID(w);
	NewtonMaterialSetContinuousCollisionMode(w, group, group, 1);
	if (target == 0)
	{
		NewtonCollision* tree = NewtonCreateTreeCollision(w, NULL);
		NewtonTreeCollisionBeginBuild(tree);
		float a[3] = {1, -2, 2}, b[3] = {1, 2, 2}, c[3] = {1, 2, -2}, d[3] = {1, -2, -2};
		AddQuad(tree, a, b, c, d);   // wall at x = 1, facing -x
		NewtonTreeCollisionEndBuild(tree, 0);
		LevelBody(w, tree);
	}
	else
	{
		Box(w, 0.01f, 1, 1, 0, 1, 0, 0);
	}
	NewtonCollision* shape;
	if (compound)
	{
		NewtonCollision* parts[2] = {NewtonCreateBox(w, 0.04f, 0.1f, 0.04f, NULL), NewtonCreateSphere(w, 0.03f, 0.03f, 0.03f, NULL)};
		shape = NewtonCreateCompoundCollision(w, 2, parts);
		NewtonReleaseCollision(w, parts[0]);
		NewtonReleaseCollision(w, parts[1]);
	}
	else
	{
		shape = NewtonCreateCylinder(w, 0.033f, 0.12f, NULL);
	}
	NewtonBody* can = NewtonCreateBody(w, shape);
	float m[16];
	Translation(m, 0.5f, 0, 0);
	NewtonBodySetMatrix(can, m);
	float inertia[3], origin[3];
	NewtonConvexCollisionCalculateInertialMatrix(shape, inertia, origin);
	NewtonBodySetMassMatrix(can, 0.2f, inertia[0] * 0.2f, inertia[1] * 0.2f, inertia[2] * 0.2f);
	NewtonReleaseCollision(w, shape);
	NewtonBodySetForceAndTorqueCallback(can, NoForce);
	NewtonBodySetAutoFreeze(can, 0);
	NewtonBodySetLinearDamping(can, 0);
	NewtonBodySetContinuousCollisionMode(can, continuous);
	float v[3] = {speed, 0, 0};
	NewtonBodySetVelocity(can, v);
	Step(w, 30);
	const float x = BodyX(can);
	NewtonDestroy(w);
	return x;
}

static void TestContinuousCollision()
{
	Section("Continuous collision");
	Check("can at 20 m/s bounces off a level wall", FireAt(0, 20, 1, false) < 0.4f);
	Check("can at 50 m/s bounces off a level wall", FireAt(0, 50, 1, false) < 0.4f);
	Check("can at 20 m/s bounces off a 1 cm thin box", FireAt(1, 20, 1, false) < 0.4f);
	Check("two-part body at 25 m/s bounces off a level wall", FireAt(0, 25, 1, true) < 0.4f);
	Check("with the body's switch off, a can at 20 m/s passes through the wall", FireAt(0, 20, 0, false) > 1.5f);
}

//-----------------------------------------------------------------------------
// Ray casts while a map loads
//-----------------------------------------------------------------------------

// HPL1 builds its AI node graph (cAINodeContainer::Compile) with many ray
// casts while the map loads, before the first NewtonUpdate. Bodies added
// since the last update sit in a tree that has not been rebalanced and can be
// hundreds of levels deep; the ray walk used to overflow its 256-entry stack
// there and crash (spider caves, level01_12).
static int gRayHits = 0;
static float CountRayHits(const NewtonBody*, const float*, int, void*, float) { gRayHits++; return 1; }
static unsigned RayPrefilterAll(const NewtonBody*, const NewtonCollision*, void*) { return 1; }

static int RayHitsInRow(int count, bool update)
{
	NewtonWorld* w = NewtonCreate(NULL, NULL);
	NewtonCollision* box = NewtonCreateBox(w, 0.2f, 0.2f, 0.2f, NULL);
	float m[16];
	for (int i = 0; i < count; i++)
	{
		NewtonBody* b = NewtonCreateBody(w, box);
		Translation(m, 0, 0, i * 0.004f);
		NewtonBodySetMatrix(b, m);
	}
	NewtonReleaseCollision(w, box);
	if (update) Step(w, 1);
	const float p0[3] = {0, 0, -1};
	const float p1[3] = {0, 0, 5};
	gRayHits = 0;
	NewtonWorldRayCast(w, p0, p1, CountRayHits, NULL, RayPrefilterAll);
	NewtonDestroy(w);
	return gRayHits;
}

static void TestRayCastBeforeUpdate()
{
	Section("Ray casts before the first update");
	Check("ray through 1000 bodies added this frame hits all of them", RayHitsInRow(1000, false) == 1000);
	Check("ray through 1000 bodies after an update hits all of them", RayHitsInRow(1000, true) == 1000);
}

int main()
{
	setvbuf(stdout, NULL, _IONBF, 0);
	printf("Newton 1.53 layer test (headless, 60 Hz steps)\n");
	TestBuffers();
	TestCollisionCollide();
	TestJointReadouts();
	TestBodies();
	TestConeLimits();
	TestLimitCallbacks();
	TestMaterials();
	TestJointUtilities();
	TestSafety();
	TestLevelMesh();
	TestContinuousCollision();
	TestOffsetInertia();
	TestRayCastBeforeUpdate();
	printf("\n%d of %d checks passed%s\n", gChecks - gFailures, gChecks, gFailures ? "" : ": all good");
	return gFailures ? 1 : 0;
}
