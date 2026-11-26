# AngelScript 2.7.1b (vendored)

The scripting library HPL1Engine was written against, the same version the
original Penumbra Overture release shipped with.

- **Source:** AngelScript SourceForge SVN, `branches/2.7.1/sdk/angelscript`
  at revision 90 (https://sourceforge.net/p/angelscript/code/90/tree/branches/2.7.1/).
  Only `include/` and `source/` are copied.
- **Licence:** zlib (see `LICENSE`, also at the top of `include/angelscript.h`).
- **Local changes:** none in the initial import. Every later change is a
  separate commit, marked in the code with `HPL1Engine:` comments (as the
  zlib licence requires for altered versions) and listed below.

## Local changes

_None yet._

## Build notes

Always built with `AS_MAX_PORTABILITY` (see `CMakeLists.txt` in this folder).
On 64-bit there's no native calling-convention support in 2.7.1b anyway, and
on 32-bit x86 portability mode avoids the stack-misalignment bug in the
native-call assembly (the AMD/Mesa "coin in the vice" crash).
