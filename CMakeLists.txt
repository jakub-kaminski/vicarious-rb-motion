cmake_minimum_required(VERSION 3.8)

project(robot-arm-wasm)

set(CMAKE_VERBOSE_MAKEFILE on)

set(CMAKE_CXX_STANDARD 17)

#set(CMAKE_EXE_LINKER_FLAGS "-s USE_BOOST_HEADERS=1")

add_subdirectory(lib)
add_subdirectory(test)
add_subdirectory(wasm)

set(INCLUDES
        lib/RBMotion/include
        lib/URDF_Parser/include
        )

include_directories(
        /lib/rigid-body-motion/lib/URDF_Parser/include
        /lib/trajcdectory/lib/eigen
)
