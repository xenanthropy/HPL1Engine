cmake_minimum_required (VERSION 3.16)
project (HPL)

# The engine was written against C++98/03. Pin the standard explicitly
# instead of relying on the compiler's default (gnu++17 on GCC 13).
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(HPL_VERSION_MAJOR 1)
set(HPL_VERSION_MINOR 0)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)
find_package(SDL_ttf REQUIRED)
find_package(Cg REQUIRED)

file(GLOB_RECURSE all_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/sources/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/sources/*.c
)
# Removed Sources
file(GLOB_RECURSE to_remove_all
    win32.cpp
    Video*Theora.cpp
    )
list(REMOVE_ITEM all_sources
    ${to_remove_all})
list(REMOVE_DUPLICATES all_sources)

add_library(HPL STATIC ${all_sources})

# Newton 1.53 API, built from vendored source (replaces the old prebuilt
# 32-bit libNewton.a). Two cores are available while we compare them:
#   4   - Julio Jerez's compatibility layer on the Newton 4 core (default)
#   236 - our 1.53 API on the Newton 2.36 core (ScummVM's copy)
set(HPL1_NEWTON_CORE "4" CACHE STRING "Newton core under the 1.53 API: 4 or 236")
set_property(CACHE HPL1_NEWTON_CORE PROPERTY STRINGS 4 236)
if(HPL1_NEWTON_CORE STREQUAL "4")
    add_subdirectory(external/newton-1.53)
    set(HPL1_NEWTON_TARGET newton153)
elseif(HPL1_NEWTON_CORE STREQUAL "236")
    add_subdirectory(external/newton-2.36)
    set(HPL1_NEWTON_TARGET newton236)
else()
    message(FATAL_ERROR "HPL1_NEWTON_CORE must be 4 or 236 (got '${HPL1_NEWTON_CORE}')")
endif()
message(STATUS "HPL1Engine: Newton core ${HPL1_NEWTON_CORE}")

# AngelScript 2.7.1b, built from the vendored source.
add_subdirectory(external/angelscript)

# The OALWrapper target is defined by the top-level project, which adds
# ../OALWrapper before this directory.
if(NOT TARGET OALWrapper)
    message(FATAL_ERROR "HPL needs the OALWrapper target; add ../OALWrapper first")
endif()

# Sources include SDL as "SDL/SDL.h", so also add the parent of SDL's
# include folder.
get_filename_component(SDL_PARENT_INCLUDE_DIR ${SDL_INCLUDE_DIR} DIRECTORY)

target_include_directories(HPL
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${SDL_INCLUDE_DIR}
        ${SDL_PARENT_INCLUDE_DIR}
        ${SDL_IMAGE_INCLUDE_DIRS}
        ${SDL_TTF_INCLUDE_DIRS}
)

target_link_libraries(HPL
    PUBLIC
        OALWrapper
        ${HPL1_NEWTON_TARGET}
        angelscript
        Cg::Cg
        GLEW::GLEW
        OpenGL::GLU
        OpenGL::GL
        ${SDL_TTF_LIBRARIES}
        ${SDL_IMAGE_LIBRARIES}
        ${SDL_LIBRARY}
)
