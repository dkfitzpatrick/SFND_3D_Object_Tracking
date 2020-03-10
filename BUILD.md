
# Build Instructions

mkdir build
cd build
cmake -DOPENCV_ENABLE_NONFREE:BOOL=ON -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules/ -DCMAKE_BUILD_TYPE=RELEASE ..

cmake -DOPENCV_ENABLE_NONFREE:BOOL=ON -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules/ -DCMAKE_BUILD_TYPE=DEBUG ..

# second try
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -D OPENCV_ENABLE_NONFREE=ON ..

# something to attempt to resolve debug issue
cmake -D BUILD_TIFF=ON -D WITH_CUDA=OFF -D ENABLE_AVX=OFF -D WITH_OPENGL=ON -D WITH_OPENCL=OFF -D WITH_IPP=OFF -D WITH_TBB=ON -D BUILD_TBB=ON -D WITH_EIGEN=OFF -D WITH_V4L=OFF -D WITH_VTK=OFF -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -D OPENCV_ENABLE_NONFREE=ON ..

------------------

// change the following to use a fullpath to the root of the dat directory.

string dataPath = "../";

------------------
Git LFS

https://www.atlassian.com/git/tutorials/git-lfs
https://git-lfs.github.com/
https://github.com/git-lfs/git-lfs/wiki/Installation


    % curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
    % sudo apt-get install git-lfs
    % git lfs install

    # pull datafiles into cache
    % git lfs pull


