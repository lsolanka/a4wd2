# Copyright (c) 2013-2017, Ruslan Baratov
# All rights reserved.

# Do not place header guards here

# Unset:
#   * ${PACKAGE_NAME}_ROOT (CMake variable)
#   * ${PACKAGE_NAME}_ROOT (CMake cache variable)
#   * ${PACKAGE_NAME}_ROOT (environment variable)

# Set CMake variables:
#   * HUNTER_${PACKAGE_NAME}_VERSION
#   * HUNTER_${PACKAGE_NAME}_CMAKE_ARGS (optionally)

# Usage:
#   hunter_config(Foo VERSION 1.0.0)
#   hunter_config(Boo VERSION 1.2.3z CMAKE_ARGS BOO_WITH_A=ON)

# Wiki:
#   * https://github.com/ruslo/hunter/wiki/dev.modules#hunter_config

include(hunter_config)
include(hunter_user_error)

# NOTE: no names with spaces!

hunter_config(Boost VERSION 1.65.1)
if(CMAKE_CROSSCOMPILING)
    hunter_config(MRPT VERSION ${HUNTER_MRPT_VERSION}
        CMAKE_ARGS
            BUILD_TESTING=OFF
            BUILD_APPLICATIONS=OFF
            DISABLE_PYTHON_BINDINGS=ON
            EIGEN_USE_EMBEDDED_VERSION=OFF
            DISABLE_VTK=ON
            DISABLE_WXWIDGETS=ON
            DISABLE_OPENGL=ON
            DISABLE_PCL=ON
            USE_QT=OFF 
            MRPT_OPTIMIZE_NATIVE=OFF
    )
    hunter_config(OpenCV VERSION ${HUNTER_OpenCV_VERSION}
        CMAKE_ARGS
            WITH_OPENEXR=OFF
            WITH_JASPER=OFF
            WITH_WEBP=OFF
            WITH_QT=OFF
            WITH_OPENGL=OFF
            WITH_IPP=OFF
            WITH_VTK=OFF
    )
else()
    hunter_config(MRPT VERSION ${HUNTER_MRPT_VERSION}
        CMAKE_ARGS
            BUILD_TESTING=OFF
            BUILD_APPLICATIONS=OFF
            DISABLE_PYTHON_BINDINGS=ON
            EIGEN_USE_EMBEDDED_VERSION=OFF
            DISABLE_VTK=ON
            DISABLE_OPENGL=ON
            DISABLE_PCL=ON
            USE_QT=OFF 
    )
    hunter_config(OpenCV VERSION ${HUNTER_OpenCV_VERSION}
        CMAKE_ARGS
            WITH_OPENEXR=OFF
            WITH_JASPER=OFF
            WITH_WEBP=OFF
            WITH_QT=ON
            WITH_OPENGL=ON
            WITH_IPP=OFF
            WITH_VTK=OFF
    )
endif()

hunter_config(roboclaw GIT_SUBMODULE "submodules/roboclaw")
