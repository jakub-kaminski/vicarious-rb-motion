#!/bin/sh
# Script for transcompiling to C++ to .WASM using the Emscripten Toolchain

js_dir="/home/jakub/Documents/robotsim-org/robot-simulator-web"

rm -f ${js_dir}/public/moduleVicarious.wasm
rm -rf ${js_dir}/public/moduleVicarious.worker.js
rm -f ${js_dir}/src/simulations/RobotArm/moduleVicarious.mjs

#rm -rf $(pwd)/build-emscripten
#mkdir -p $(pwd)/build-emscripten

emcmake cmake -B $(pwd)/build-emscripten -S $(pwd) -DCMAKE_BUILD_TYPE=Release
cmake --build $(pwd)/build-emscripten --target moduleVicarious -j 8

mv $(pwd)/build-emscripten/wasm/moduleVicarious.wasm ${js_dir}/public/moduleVicarious.wasm
mv $(pwd)/build-emscripten/wasm/moduleVicarious.mjs ${js_dir}/src/simulations/RobotArm/moduleVicarious.mjs
