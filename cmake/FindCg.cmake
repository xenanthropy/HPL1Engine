# FindCg.cmake
#
# Finds the NVIDIA Cg toolkit (runtime + OpenGL runtime).
#
# Result variables:
#   Cg_FOUND
#   Cg_INCLUDE_DIR   - directory containing Cg/cg.h
#   Cg_LIBRARY       - the Cg runtime library
#   CgGL_LIBRARY     - the Cg OpenGL runtime library
#
# Imported targets:
#   Cg::Cg           - Cg + CgGL, with include directory

find_path(Cg_INCLUDE_DIR Cg/cg.h
    HINTS ENV CG_INC_PATH
    PATHS /usr/include /usr/local/include /opt/nvidia-cg-toolkit/include
          "C:/Program Files/NVIDIA Corporation/Cg/include"
          "C:/Program Files (x86)/NVIDIA Corporation/Cg/include"
          /Library/Frameworks/Cg.framework/Headers
)

find_library(Cg_LIBRARY NAMES Cg cg
    HINTS ENV CG_LIB_PATH ENV CG_LIB64_PATH
    PATHS /opt/nvidia-cg-toolkit/lib /opt/nvidia-cg-toolkit/lib64
          "C:/Program Files/NVIDIA Corporation/Cg/lib.x64"
          "C:/Program Files (x86)/NVIDIA Corporation/Cg/lib"
)

find_library(CgGL_LIBRARY NAMES CgGL cgGL
    HINTS ENV CG_LIB_PATH ENV CG_LIB64_PATH
    PATHS /opt/nvidia-cg-toolkit/lib /opt/nvidia-cg-toolkit/lib64
          "C:/Program Files/NVIDIA Corporation/Cg/lib.x64"
          "C:/Program Files (x86)/NVIDIA Corporation/Cg/lib"
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Cg
    REQUIRED_VARS Cg_LIBRARY CgGL_LIBRARY Cg_INCLUDE_DIR)

if(Cg_FOUND AND NOT TARGET Cg::Cg)
    add_library(Cg::Cg INTERFACE IMPORTED)
    set_target_properties(Cg::Cg PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${Cg_INCLUDE_DIR}"
        INTERFACE_LINK_LIBRARIES "${CgGL_LIBRARY};${Cg_LIBRARY}")
endif()

mark_as_advanced(Cg_INCLUDE_DIR Cg_LIBRARY CgGL_LIBRARY)
