# Yukari

[![Build Status](https://travis-ci.com/DanNixon/Yukari.svg?token=hEeXj1er91qf6vBmhf9x&branch=master)](https://travis-ci.com/DanNixon/Yukari)

3D environment mapper

## Building on Ubuntu 16.04

1. Install `libproj-dev libboost-all-dev libpcl1.7 libpcl-dev`
2. `cmake ../Yukari -DCMAKE_BUILD_TYPE=Debug -DBOOST_LIBRARYDIR=/usr/lib/x86_64-linux-gnu/`
3. `make`

## Building on Windows

1. Install a prebuilt version of Boost
2. Install prebuilt PCL and dependencies: http://pointclouds.org/downloads/windows.html
3. `cmake ../Yukari -G "Visual Studio 14 2015 Win64" -DCMAKE_BUILD_TYPE=Debug -DBoost_INCLUDE_DIR=C:\\local\\boost_1_63_0 -DBoost_LIBRARY_DIR=C:\\local\\boost_1_63_0\\lib64-msvc-14.0`
