cmake_minimum_required (VERSION 3.16)
project (HPL)

# The engine was written against C++98/03. Pin the standard explicitly
# instead of relying on the compiler's default (gnu++17 on GCC 13).
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(HPL_VERSION_MAJOR 1)
set(HPL_VERSION_MINOR 0)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/../OALWrapper/include
	${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/include
)
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
set_target_properties(Newton PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/lib/linux/libNewton.a)

add_library(angelscript STATIC IMPORTED)
set_target_properties(angelscript PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/lib/linux/libangelscript.a)

add_library(SDLmain STATIC IMPORTED)
set_target_properties(SDLmain PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/lib/linux/libSDLmain.a)
