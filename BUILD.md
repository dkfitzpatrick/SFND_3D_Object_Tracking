
# Build Instructions

mkdir build
cd build
cmake -DOPENCV_ENABLE_NONFREE:BOOL=ON -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules/ -DCMAKE_BUILD_TYPE=RELEASE ..

cmake -DOPENCV_ENABLE_NONFREE:BOOL=ON -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules/ -DCMAKE_BUILD_TYPE=DEBUG ..
