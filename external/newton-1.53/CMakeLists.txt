# Newton 1.53 compatibility layer on the Newton 4 core, built as a static
# library for HPL1Engine. Replaces the old prebuilt 32-bit libNewton.a.
# (Our own file; the upstream CMakeLists.txt is MSVC-only and not vendored.)

file(GLOB newton153_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/joints/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/newton-4/source/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/newton-4/source/*.c
)

add_library(newton153 STATIC ${newton153_sources})

# Newton 4 needs C++14 or newer. This applies to this target only: HPL and
# the game stay on C++11 and only ever see the plain C header Newton.h.
set_target_properties(newton153 PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
)

# Newton.h lives at the top level. The core's headers stay private.
target_include_directories(newton153
    PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/newton-4/source
)

# Static library: Newton.h then uses plain NEWTON_API on every platform
# (it already defaults to this on non-MSVC compilers).
target_compile_definitions(newton153 PUBLIC _NEWTON_USE_LIB)
target_compile_definitions(newton153 PRIVATE _CRT_SECURE_NO_WARNINGS)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # Same as upstream: GCC and Clang use the portable scalar math path
    # instead of the hand-written SSE vector class.
    target_compile_definitions(newton153 PRIVATE D_SCALAR_VECTOR_CLASS)
    # Third-party code: keep its warnings out of our build output.
    target_compile_options(newton153 PRIVATE -w)
    # The solver is very slow unoptimised; optimise it even in Debug builds.
    target_compile_options(newton153 PRIVATE $<$<CONFIG:Debug>:-O2>)
elseif(MSVC)
    target_compile_options(newton153 PRIVATE /w $<$<CONFIG:Debug>:/O2>)
endif()

# The Newton 4 core runs its update on worker threads.
find_package(Threads REQUIRED)
target_link_libraries(newton153 PUBLIC Threads::Threads)
