#!/bin/sh

mkdir -p build
cd ./build
#cmake -DCMAKE_BUILD_TYPE=Debug ..
cmake ..
make
./multitrack "file:../input/track_v2025.txt" "../input/rect_v2025.txt" | tee ../output/console.txt
cd ..