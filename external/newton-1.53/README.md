# Newton 1.53 compatibility layer (vendored)

Julio Jerez's Newton Game Dynamics 1.53 API, implemented on top of a
self-contained copy of the Newton 4 core. HPL1Engine was written against the
1.53 API, which only ever shipped as closed, mostly 32-bit binaries.

- Upstream: https://github.com/JulioJerez/newton-dynamics
- Path: `newton-4.00/applications/toolsAndWrapers/newton-1.53`
- Commit: `294f27f6cde10c3c269041c3c4944cefb0480dbf`
- Licence: zlib (see `LICENSE`, copied from the repository root)

## What was copied

Everything in that directory except `demos/` (23 MB of tutorials) and the
upstream `CMakeLists.txt` (MSVC-only). Our own `CMakeLists.txt` replaces it.

- top-level `Newton*.cpp/.h`, `newtonWorld`, `newtonStdafx`: the 1.53 API
- `joints/`: the 1.53 joint types
- `newton-4/source/`: the Newton 4 core

## Local changes

The layer is incomplete for HPL1Engine's needs; we finish it here. **These
changes are local to this fork and are never sent upstream.** Each one is
marked with an `// HPL1Engine:` comment and logged in PenumbraOverture's
`FIXES.md`.

Where Newton 4 defaults and real 1.53 behaviour disagree, we choose 1.53
behaviour, to keep the game's physics feel.

1. GCC portability (no behaviour change):
   - `NewtonUtils.h`: `_finite`/`_isnan` (MSVC-only) replaced with
     `std::isfinite`/`std::isnan` (only compiled when `_DEBUG` is defined).
   - `NewtonVehicle.cpp`: removed an extra class qualifier on
     `PostTransformUpdate` inside the class body.
   - `joints/NewtonJoints.cpp`: the new joint user data is put in a local
     before `SetUserData`, which takes a non-const reference.
2. Matrix copy overrun on GCC/Clang (memory corruption, real bug):
   `ndMatrix(const ndFloat32*)` in `newton-4/source/ndMatrix.h`,
   `NewtonBodyGetMatrix` and `NewtonConvexHullModifierGetMatrix` copied
   `sizeof(ndMatrix)/sizeof(ndFloat32)` floats. Under the Itanium C++ ABI
   (GCC, Clang) `ndMatrix` is 96 bytes with `m_front` at offset 16, because
   it and its `ndVector` members share the empty base `ndClassAlloc`; MSVC
   lays it out as 64 bytes. So 24 floats were copied instead of 16, writing
   past the matrix or the caller's 16-float buffer. Now exactly 16.
3. Per-body freeze thresholds: `NewtonBody.h` adds `ndNewtonBodyDynamic`
   (used by `NewtonCreateBody`), which applies the speed^2/omega^2 limits from
   `NewtonBodySetFreezeTreshold` in `EvaluateSleepState`. In the core,
   `ndBodyDynamic::EvaluateSleepState` moved from private to protected so the
   subclass can call it.
4. `NewtonHPL1Engine.cpp` (new) and a block at the end of `Newton.h`: the 12
   1.53 functions HPL1Engine calls that the layer declared without defining:
   `NewtonCollisionCollide`, `NewtonBall/Hinge/Slider/CorkscrewGetJointForce`,
   `NewtonBallGetJointAngle/Omega`, `NewtonMaterialGetContactForce`,
   `NewtonBodySetFreezeTreshold`, `NewtonSetSolverModel`,
   `NewtonSetFrictionModel`, `NewtonMaterialSetContinuousCollisionMode`.
5. Stubs and behaviour fixes (Phase 5, step 5):
   - `NewtonBodyGetSleepingState` returned 1 for sleeping; 1.53 returns 1
     for active. Inverted.
   - `NewtonAddBodyImpulse` (was empty): 1.53's "change the velocity of this
     point by this much" impulse, from 2.36's `dgBody::AddImpulse`.
   - `NewtonWorldFreezeBody` (was empty): clears velocity and sleeps the body.
   - `NewtonHinge/Slider/CorkscrewCalculateStop*` (returned 0): 1.53/2.36's
     formulas, in `NewtonDeprecated.cpp`.
   - `NewtonBallSetConeLimits`: uses the given cone axis (was ignored), with
     1.53's thresholds and clamps.
   - `NewtonBodySet/GetContinuousCollisionMode`: the flag is stored on the
     body and reported back (no simulation effect, as in 1.53).

6. Joint limit callbacks (Phase 5, step 6): the hinge, slider and corkscrew
   wrappers called HPL1's callback but ignored what it returned, so no door,
   lever, drawer or screw limit had any effect. Now, as in 1.53/2.36, a
   non-zero return adds one row along the pin after the base rows (angular
   for the hinge, linear for the slider; corkscrew bit 0 = linear row from
   `desc[0]`, bit 1 = angular row from `desc[1]`), with the callback's
   acceleration and its min/max force as bounds. The defaults are
   unbounded (were ±1e-20). The joint's angle/position are refreshed before
   the callback. Helpers in `joints/NewtonHPL1Joints.h`.
7. Contacts and the world update (Phase 5, step 7):
   - Contacts pick their material from the two bodies' 1.53 material groups
     (`newtonWorld.cpp`). The core looked it up by shape user ids, which the
     layer never set, so every contact used the default material and no
     HPL1 contact callback ever ran. Pairs with group 0 get their own
     material too.
   - The core is built with `D_USE_THREAD_EMULATION` and one thread, so
     `NewtonUpdate` runs on the calling thread, as in 1.53, and a pair's
     begin/process/end callbacks run back to back. (`ndThreadPool.h`: the
     emulation branch of `ParallelExecute` was fixed to compile.)
   - One substep per update instead of 3.
   - The standard solver instead of SIMD8, which mixed up the rows of
     constraints with different row counts in one batch.
   - `ndScene.cpp`: a pair rejected by the begin callback drops its
     contacts; contacts of moving pairs are recomputed every step (no
     contact cache, `HPL1_CONTACT_CACHE`).
   - Elasticity clamped to 0.01..2, as in 1.53 (was 0..1).
8. Linking into HPL1Engine (Phase 5, step 8):
   - `NewtonDestroyJoint`, `NewtonJointGetCollisionState` and
     `NewtonJointGetStiffness` only existed inside `#if 0`; defined in
     `NewtonHPL1Engine.cpp`. `NewtonJointSetCollisionState` asserted and
     did nothing; it now sets the joint's collidable flag.
     `NewtonJointSetStiffness` stores the value (default 0.9, as in 1.53)
     so it can be read back; it has no effect on Newton 4.
   - `Newton.h` includes `<stddef.h>` for `size_t`.
9. Safety (Phase 6): `ndNewtonWorld::GuardBodies`, run after each update.
   - 1.53's world box. `NewtonSetWorldSize` was ignored; the layer now keeps
     the box (default +-100, as at 1.53 world creation) and, as 1.53 did,
     disables a body whose bounding box leaves it: velocity zeroed and put to
     sleep, again each update while it stays outside. Without it, a thrown
     object flew on for ever (the boat lamp in level 1).
   - NaN guard: a body whose matrix or velocity becomes NaN/inf is put back
     at its last valid matrix and stopped, with one message on stderr.
10. Spin cap (Phase 6): `ndBodyKinematic::IntegrateVelocity` never turns a
    body more than 45 degrees in one step, shrinking its angular velocity by
    0.8 until it fits, as Newton 2.x's `dgBody::IntegrateVelocity` did (and,
    we assume, 1.53). Newton 4 only asserts this in debug builds. Without it
    Penumbra's grab damping, which overshoots on a thin body, made the boat
    lamp spin up without bound and fly away. Only bodies spinning faster
    than about 47 rad/s at 60 Hz are affected.

## Test

`tests/newton153_test.cpp` is a headless test of the layer as HPL1Engine
uses it: 61 checks covering buffers, shape collision, joint readouts and
stop helpers, sleeping/impulse/freeze, ball cone limits, HPL1-style joint
limit callbacks, materials and contact callbacks, joint utilities, and
the world box, NaN guard and spin cap. It
draws nothing, so it runs the same on any machine. From the build folder:

```sh
make newton153_test && ./HPL1Engine/external/newton-1.53/newton153_test
```

Exit code 0 means every check passed; each check prints PASS or FAIL.
