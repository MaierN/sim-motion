
# Sim motion
Use your mouse + click to move the robot

Red robot = physical position\
Pink robot = odometry only\
Green robot = odometry + lidar Monte Carlo

Blue line = static obstacle\
Green line = dynamic obstacle

Red point = lidar measure considered as a wall\
Pink point = lidar measure considered as a dynamic obstacle

Error is shown in the upper left corner

Particles are shown as black/pink arrows based on their weight

## Build
```
mkdir build
cd build
cmake ..
cmake --build .
```

## Run simulation
```
cd build
./sim-motion
```
