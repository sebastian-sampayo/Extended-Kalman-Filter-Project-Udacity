# Extended Kalman Filter Project
In this project I implement an Extended Kalman Filter to make Sensor Fusion with Radar and Laser measurements.
The code is written in C++, because this language provides great performance in terms of memory and speed (if we do things the right way).
This work is part of the Self-Driving Car Engineer Nanodegree Program.

---

[//]: # (Image References)
[simulation]: ./utils/simulation.png

## Simulation
In order to test the program I used a visualization tool provided by Udacity that plots noisy lidar and radar measurements as a simulated car drives in a figure eight pattern.
The program can also record measurement data directly to a text file and then use a python script to call a compiled c++ Kalman Filter file to track estimated markers and RMSE values visually in real time.
![Simulator][simulation]

## Source files hierarchy and description
- main.cpp : Main program. Reads in measurements and apply Sensor Fusion algorithm.
  - FusionEKF.cpp/.h : Sensor Fusion high level class.
    - kalman_filter.cpp/.h: Kalman Filter low level class. Implements both simple and extended Kalman Filter.
    - tools.cpp/.h : Common math tools.
    - measurement_package.h : Class for measurements values.
  - ground_truth_package.h : Class for ground truth values.

## Application dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

