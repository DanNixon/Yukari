# Yukari

[![Build Status](https://travis-ci.com/DanNixon/Yukari.svg?token=hEeXj1er91qf6vBmhf9x&branch=master)](https://travis-ci.com/DanNixon/Yukari)

## Building on Ubuntu 16.04

1. Install `libproj-dev libboost-all-dev libpcl1.7 libpcl-dev libeigen3-dev`
2. `cmake ../Yukari -DCMAKE_BUILD_TYPE=Debug`
3. `make`

## Building on Windows

1. Install PCL and dependencies
     - Simplest method is the all in one installers from http://unanancyowen.com/en/pcl18/
2. `cmake ../Yukari -G "Visual Studio 14 2015 Win64" -DCMAKE_BUILD_TYPE=Debug`
     - (for MSVC 14)
