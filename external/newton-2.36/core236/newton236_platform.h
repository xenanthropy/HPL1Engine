// HPL1Engine: stands in for the ScummVM headers that ScummVM's copy of
// Newton 2.36 included (common/scummsys.h, common/util.h, common/system.h,
// hpl1/debug.h). Only the few names the core uses are defined here.
#ifndef NEWTON236_PLATFORM_H
#define NEWTON236_PLATFORM_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <new>

typedef int8_t int8;
typedef uint8_t uint8;
typedef int16_t int16;
typedef uint16_t uint16;
typedef int32_t int32;
typedef uint32_t uint32;
typedef int64_t int64;
typedef uint64_t uint64;
typedef uint8_t byte;
typedef unsigned int uint;

#ifndef ARRAYSIZE
#define ARRAYSIZE(x) ((int)(sizeof(x) / sizeof(x[0])))
#endif

#ifndef FORCEINLINE
#if defined(_MSC_VER)
#define FORCEINLINE __forceinline
#else
#define FORCEINLINE inline __attribute__((always_inline))
#endif
#endif

#define HPL1_UNIMPLEMENTED(x) fprintf(stderr, "Newton 2.36: %s is not implemented\n", #x)

#endif
