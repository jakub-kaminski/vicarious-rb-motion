add_executable(moduleVicarious
        moduleVicarious.cpp
        ../src/Vicarious.cpp
        ../src/VicariousRobot.cpp
        ../src/VicariousUtils.cpp
        )

target_include_directories(moduleVicarious PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/lib/URDF_Parser/include
        ${CMAKE_SOURCE_DIR}/lib/rigid-body-motion/include
        ${CMAKE_SOURCE_DIR}/lib/Trajectory/include
        ${CMAKE_SOURCE_DIR}/include
        )

#set(CMAKE_CXX_FLAGS "-s TOTAL_MEMORY=1024MB")
message(STATUS ${EM_FLAGS})

# Turn on shared array buffers on the web applications

if(EMSCRIPTEN)
set(EM_FLAGS "${EM_FLAGS} -lembind -O3 --no-entry -v")

set(EM_FLAGS "${EM_FLAGS} -s USE_BOOST_HEADERS=1")

set(EM_FLAGS "${EM_FLAGS} -s ENVIRONMENT='web'")
set(EM_FLAGS "${EM_FLAGS} -s ALLOW_MEMORY_GROWTH=1")

set(EM_FLAGS "${EM_FLAGS} --pre-js ${CMAKE_SOURCE_DIR}/wasm/locateFile.js")
set(EM_FLAGS "${EM_FLAGS} -s EXPORT_NAME=\"importVicarious\"")
set(EM_FLAGS "${EM_FLAGS} -s USE_ES6_IMPORT_META=0")
#set(EM_FLAGS "${EM_FLAGS} -s EXPORTED_RUNTIME_METHODS='['ccall', 'cwrap', '_js_get_global']")
set(EM_FLAGS "${EM_FLAGS} -s EXPORTED_RUNTIME_METHODS='['ccall', 'cwrap']'")
set(EM_FLAGS "${EM_FLAGS} --no-entry -v")

set(CMAKE_EXECUTABLE_SUFFIX ".mjs")
endif()

target_link_libraries(moduleVicarious RBMotion urdfparser Trajectory)

if(EMSCRIPTEN)
set_target_properties(moduleVicarious PROPERTIES
        LINK_FLAGS "${EM_FLAGS}"
        COMPILE_FLAGS  "${EM_FLAGS}" )
endif()
