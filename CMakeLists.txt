cmake_minimum_required (VERSION 3.16)
project (HPL)

# The engine was written against C++98/03. Pin the standard explicitly
# instead of relying on the compiler's default (gnu++17 on GCC 13).
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(HPL_VERSION_MAJOR 1)
set(HPL_VERSION_MINOR 0)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Location of the historical prebuilt dependency bundle. Only AngelScript and
# Newton still come from here; both are replaced by source builds later
# (see ROADMAP.md, Phases 4 and 5).
set(HPL_DEPENDENCIES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../dependencies"
    CACHE PATH "Folder holding the prebuilt AngelScript/Newton headers and libs")

find_package(OpenGL REQUIRED)
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

add_library(Newton STATIC IMPORTED)
set_target_properties(Newton PROPERTIES
    IMPORTED_LOCATION ${HPL_DEPENDENCIES_DIR}/lib/linux/libNewton.a)

add_library(angelscript STATIC IMPORTED)
set_target_properties(angelscript PROPERTIES
    IMPORTED_LOCATION ${HPL_DEPENDENCIES_DIR}/lib/linux/libangelscript.a)

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
        ${HPL_DEPENDENCIES_DIR}/include
        ${SDL_INCLUDE_DIR}
        ${SDL_PARENT_INCLUDE_DIR}
        ${SDL_IMAGE_INCLUDE_DIRS}
        ${SDL_TTF_INCLUDE_DIRS}
)

target_link_libraries(HPL
    PUBLIC
        OALWrapper
        Newton
        angelscript
        Cg::Cg
        OpenGL::GLU
        OpenGL::GL
        ${SDL_TTF_LIBRARIES}
        ${SDL_IMAGE_LIBRARIES}
        ${SDL_LIBRARY}
        fltk
)
