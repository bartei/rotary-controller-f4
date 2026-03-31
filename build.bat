mkdir build
cd build
cmake -G Ninja -DCMAKE_BUILD_TYPE=Release ..
ninja
st-flash --format ihex write rotary-controller-f4.hex
cd ..
rmdir build /S /Q