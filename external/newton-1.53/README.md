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
