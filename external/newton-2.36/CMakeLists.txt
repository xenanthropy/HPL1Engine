# The Newton 1.53 API that HPL1Engine uses, on the Newton Game Dynamics 2.36
# core (ScummVM's copy, in core236/). Built when HPL1_NEWTON_CORE is 236.
# (Our own file.)

file(GLOB_RECURSE newton236_core_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/core236/*.cpp
)

add_library(newton236 STATIC
    ${newton236_core_sources}
    ${CMAKE_CURRENT_SOURCE_DIR}/newton153_on_236.cpp
)

set_target_properties(newton236 PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)

# HPL1 includes "Newton.h" and gets the 1.53 API. The core's headers stay
# private; its own API is renamed N236_ so the two never clash.
target_include_directories(newton236
    PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/core236
)
target_compile_definitions(newton236 PUBLIC _NEWTON_USE_LIB)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # Third-party code: keep its warnings out of our build output.
    target_compile_options(newton236 PRIVATE -w)
    # The solver is very slow unoptimised; optimise it even in Debug builds.
    target_compile_options(newton236 PRIVATE $<$<CONFIG:Debug>:-O2>)
elseif(MSVC)
    target_compile_options(newton236 PRIVATE /w $<$<CONFIG:Debug>:/O2>)
endif()

# The same headless test as the Newton 4 layer's, run against this core:
#     make newton236_test && ./HPL1Engine/external/newton-2.36/newton236_test
add_executable(newton236_test EXCLUDE_FROM_ALL ../newton-1.53/tests/newton153_test.cpp)
target_link_libraries(newton236_test PRIVATE newton236)
set_target_properties(newton236_test PROPERTIES CXX_STANDARD 11 CXX_STANDARD_REQUIRED ON)
