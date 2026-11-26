# AngelScript 2.7.1b, built as a static library for HPL1Engine.

file(GLOB angelscript_sources ${CMAKE_CURRENT_SOURCE_DIR}/source/*.cpp)

add_library(angelscript STATIC ${angelscript_sources})

target_include_directories(angelscript PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# PUBLIC on purpose: the engine and the game must see the same setting as
# the library, so they register script functions through the generic
# calling convention. On 64-bit, 2.7.1b has no native calling support and
# would reject native registrations; on 32-bit x86, portability mode avoids
# the stack-misalignment bug in the native-call assembly.
target_compile_definitions(angelscript PUBLIC AS_MAX_PORTABILITY)

# Third-party code: keep its (many, harmless) old-compiler warnings out of
# our build output.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(angelscript PRIVATE -w)
elseif(MSVC)
    target_compile_options(angelscript PRIVATE /w)
endif()

set_target_properties(angelscript PROPERTIES POSITION_INDEPENDENT_CODE ON)
