#!/bin/bash
# tested with ubuntu 18.04 docker image in vscode remote container extension
# Note:  The code uses c++17 and libs built using other c++ version cause linker errors.
mkdir libs
cd libs
sudo apt-get update
sudo apt install libeigen3-dev -y  # this may be works because its header only.
wget https://github.com/google/protobuf/releases/download/v2.6.1/protobuf-2.6.1.tar.gz
tar -xf protobuf-2.6.1.tar.gz
cd protobuf-2.6.1/
./configure
make -j4
sudo make install
cd ..
wget https://github.com/google/glog/archive/refs/tags/v0.4.0.tar.gz
tar -xf v0.4.0.tar.gz
cd glog-0.4.0
cmake .
make -j4
sudo make install
cd ..
wget https://github.com/ceres-solver/ceres-solver/archive/refs/tags/1.14.0.tar.gz
tar -xf 1.14.0.tar.gz
cd ceres-solver-1.14.0
cmake .
make -j4 
sudo make install
cd ..
#sudo apt-get install libceres-dev -y
cd ..

