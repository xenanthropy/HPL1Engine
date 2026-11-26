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

