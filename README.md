# C++ Route Planning using A*

This project implements a A* route planner running on OpenStreetMap in C++.

It calculate the shortest path from start point (Green Colour) to goal point (Red Colour). The user enters start coordinates (X,Y) and goal coordinates (X,Y). The value should be between 0 and 100, then the shortest path is computed using A* algorithm and is visualized on map using the io2d library:

<img src="map.png" width="600" height="450"/>

The distance between the two points is printed out in the console in meters afterwards. Results section has few more combinations of start and end goal. 

## Cloning

To clone this repo, be sure to use the `--recurse-submodules` flag.
```bash
git clone https://github.com/vrushabhdesai/OpenStreetMap_Route_Planning.git --recurse-submodules
```

## Required Dependencies for Running Locally
* cmake >= 3.11.3
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 7.4.0
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same instructions as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* IO2D
  * Installation instructions for all operating systems can be found [here](https://github.com/cpp-io2d/P0267_RefImpl/blob/master/BUILDING.md)
  * This library must be built in a place where CMake `find_package` will be able to find it

## Compiling and Running

### Compiling
To compile the project, first, create a `build` directory and change to that directory:
```bash
mkdir build && cd build
```
From within the `build` directory, then run `cmake` and `make` as follows:
```bash
cmake ..
make
```
### Running
The executable will be placed in the `build` directory. From within `build`, you can run the project as follows:
```bash
./OSM_A_star_search
```
Or to specify a map file for example here it uses Worcester map. 
```bash
./OSM_A_star_search -f ../resources/Worcester_map.osm
```

## Testing

The testing executable is also placed in the `build` directory. From within `build`, you can run the unit tests as follows:
```bash
./test
```
