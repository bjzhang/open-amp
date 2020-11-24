#!/bin/bash

git fetch local
git checkout -f local/Linux-rtt-openamp
patch -p1 < rv64_build_quirk_for_cmake.patch
mkdir -p build
cd build
echo "remove old build files"
rm * -rf
echo "configration"
#cmake ../ -DWITH_APPS=ON -DWITH_LOAD_FW=ON -DLIBMETAL_LIB_DIR=../../libmetal/build/lib -DLIBMETAL_LIB="../../libmetal/build/lib/libmetal.so" -DLIBMETAL_INCLUDE_DIR=../../libmetal/build/lib/include -DWITH_RV64=ON -DWITH_STATIC_LIB=OFF -DWITH_WFI=ON -DCMAKE_C_FLAGS="-ggdb"
cmake ../ -DCMAKE_TOOLCHAIN_FILE=rv64_linux_toolchain_file
echo "build"
make DESTDIR=$PWD load_fw-shared install
# Build with verbose. For debugging compile errors
# make VERBOSE=1 DESTDIR=$PWD load_fw-shared install
