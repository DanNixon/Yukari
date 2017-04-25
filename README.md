# Yukari

[![Build Status](https://travis-ci.com/DanNixon/Yukari.svg?token=hEeXj1er91qf6vBmhf9x&branch=master)](https://travis-ci.com/DanNixon/Yukari)

## Building on Ubuntu 14.04

1. `sudo apt-get install cmake libproj-dev libboost-all-dev libeigen3-dev libpcl-1.7-all`
2. `sudo apt-get install doxygen graphviz` to also build documentation
3. `cmake ../Yukari -DCMAKE_BUILD_TYPE=Debug`
   - May need to add `-DBOOST_LIBRARYDIR=/usr/lib/x86_64-linux-gnu/` if Boost is
     not found automatically
4. `make`
5. `make doc` to build documentation

## Building on Windows

1. Install PCL and dependencies
   - Simplest method is the all in one installers from
     http://unanancyowen.com/en/pcl18/
2. `cmake ../Yukari -G "Visual Studio 14 2015 Win64" -DCMAKE_BUILD_TYPE=Debug`
   - (for MSVC 14)
