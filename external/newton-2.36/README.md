# Newton 1.53 API on the Newton Game Dynamics 2.36 core

An alternative physics core for HPL1Engine, selected at configure time:

```sh
cmake /path/to/PenumbraOverture -DHPL1_NEWTON_CORE=236   # this core
cmake /path/to/PenumbraOverture -DHPL1_NEWTON_CORE=4     # Newton 4 layer (default)
```

HPL1Engine still calls the 1.53 API (`../newton-1.53/Newton.h`). This
folder provides it on top of Newton 2.36, which is much closer to 1.53 than
Newton 4: 2.x still treats level mesh faces as two-sided thin plates with a
depth per contact point, keeps 1.x's sleep thresholds and solver models, and
reports joint angles the way 1.x did.

**Status:** comparison only. Measured against the original 1.53 library,
Newton 4 with the 1.53 layer is closer, so it stays the default. This core
still has the earlier guesses about sleeping (a 10-step quiet count, waking
on `NewtonBodySetVelocity`) that the 1.53 library disproved, and fails the
five sleep checks written from the library's behaviour.

## Where the core comes from

`core236/` is the Newton 2.36 copy in ScummVM's HPL1 port
(`engines/hpl1/engine/libraries/newton`), taken at ScummVM commit
`695524fef9945c602666a331924055091c69426e`. Newton is by Julio Jerez under
the zlib licence (see the header of each file). ScummVM's own changes to
it (formatting, 64-bit and compiler fixes) are GPLv3, like HPL1Engine.

## Our files

- `newton153_on_236.cpp`: the 1.53 API HPL1Engine uses, written on the
  2.36 API.
- `include/Newton.h`: forwards to `../newton-1.53/Newton.h`, so HPL1Engine
  includes the same header whichever core is built.
- `core236/newton236_platform.h`: integer typedefs and a few macros that
  ScummVM's `common/scummsys.h` provided.
- `CMakeLists.txt`: the `newton236` library and the `newton236_test`
  target (the Newton 4 layer's 68 checks, run against this core):

  ```sh
  make newton236_test && ./HPL1Engine/external/newton-2.36/newton236_test
  ```

## Local changes to the core

Each is marked with an `HPL1Engine:` comment where it is more than a
rename.

1. Every `Newton...` identifier in the core is renamed `N236_Newton...`
   (functions, types and callbacks), so the shim can define the 1.53 names
   in the same program. Include file names and the copyright notices are
   unchanged.
2. Includes rewritten from ScummVM's paths to `core/`, `physics/` and
   `newton236_platform.h`. `core/dgDebug.cpp` prints to stderr instead of
   ScummVM's `debug()`.
3. `Newton.cpp`: `N236_NewtonJointSetDestructor` is defined with the
   `const` joint parameter its declaration has. The non-const definition
   was a separate overload nobody could call, so the real one was missing
   at link time.
4. `core/dgPolyhedra.cpp`, `EdgePenalty`: the loop over a vertex's edges
   steps with `ptrn->m_twin->m_next` again. A ScummVM rename (commit
   `97b0b966d`) changed it to step from the wrong variable, which loops for
   ever. HPL1Engine optimises nearly every level mesh (`EndBuild(tree, 1)`),
   so this hung map loading.
5. `physics/dgBody.h` and `physics/dgWorldDynamicUpdate.cpp`: 1.53's
   sleep rule. A body may rest only after `framesCount` quiet steps in a row
   (`NewtonBodySetFreezeTreshold`; 10 by default, as 1.53 documents), and
   `NewtonWorldUnfreezeBody` starts the count again. Penumbra's boat wakes
   every body each frame for its slow wave gravity; without the count the
   lamp froze in place between waves. The helpers `HplWake`, `HplRest`,
   `HplIsActive` and `HplSetFreezeSteps` are what the shim calls.

## What the shim does beyond renaming

- 1.53 callbacks take fewer arguments than 2.36's; each body and material
  pair keeps its 1.53 callbacks in a small record and 2.36 calls a
  trampoline. Contact callbacks run begin, then process for each contact
  (removing the ones it rejects), then end, as 1.53 did.
- Matrices are copied to 16-byte aligned buffers on the way in and out:
  2.36 reads them with SSE loads, 1.53 took plain float arrays.
- `NewtonCollisionCollide` returns normals pointing from B to A, as 1.53
  did (the character controller depends on it). 2.36 does that against
  meshes but the other way round against convex and compound shapes.
- Safety, as in the Newton 4 layer: a body leaving the world box stops; a
  body whose position or velocity becomes NaN/inf goes back to its last
  valid position and stops, with one message on stderr.
- `NewtonBodySetVelocity`/`SetOmega` with a non-zero value wake a resting
  body. 2.36 leaves it asleep until something touches it. 1.53's own code
  is not available to check this; the Newton 4 layer behaves the same way.
- `NewtonBodySetFreezeTreshold`'s speed limits are not used per body:
  2.36's world thresholds already equal 1.53's for HPL1's value (0.01),
  which is what every Penumbra entity uses.
- New 2.36 bodies get linear and angular damping of about 0.1 unless told
  otherwise. HPL1Engine sets damping on every entity body, so this only
  matters for bodies it creates directly (the character body).
- Joint angles and slider positions are the ones the joint saw during the
  last step, one step behind the body, as in 1.x and 2.x.
