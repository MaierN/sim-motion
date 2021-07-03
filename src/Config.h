#pragma once

#include <random>

//#define SANITY_CHECK

const int SCREEN_WIDTH = 500;   // Width of the window
const int SCREEN_HEIGHT = 750;  // Height of the window

const int FPS = 60;                   // Upper FPS limit
const int N_LASER = 30;               // Number of measures in a full rotation of the lidar
const int N_PARTICLES = 400;          // Number of particles for the Monte Carlo simulation
const int N_SELECTED_PARTICLES = 50;  // Number of selected particles when computing the corrected position of the robot

const double ROBOT_LINEAR_SPEED = 800;  // Robot maximum linear speed
const double ROBOT_ANGULAR_SPEED = 30;  // Robot maximum angular speed
const bool ALWAYS_ROTATE = true;        // Rotate robot without mouse click

const int UPDATE_DELAY = 4;  // Number of frames between sensor update and resampling

const double WEIGHT_PHI = 20;                              // Laser sensor's noise
const double WEIGHT_DELTA = 10;                            // Small positive constant tot account for noise of the sensor (should be between 0 and WEIGHT_PHI)
const bool FILTER_MEASURES_TOO_CLOSE = false;              // Do not penalize particles if the expected distance is greater than the measured distance
const bool FILTER_MEASURES_NOT_CLOSE_TO_WALL = true;       // Do not use measures that are far from walls according to the estimated robot position
const double FILTER_MEASURES_CLOSE_TO_WALL_DISTANCE = 10;  // Distance at which a point is considered close enough to a wall

const double LASER_RANGE = 10000;  // Maximum range of lidar

#ifndef SANITY_CHECK
const double OFFSET_PARTICLE_RESAMPLE_POS = 1;        // (stddev) Position offset when resampling particles
const double OFFSET_PARTICLE_RESAMPLE_ANGLE = 0.005;  // (stddev) Angle offset when resampling particles

const double ERROR_ESTIMATED_POS = 0.1;      // (stddev) Odometry error for position
const double ERROR_ESTIMATED_ANGLE = 0.005;  // (stddev) Odometry error for angle

const double ERROR_LASER_DISTANCE = 4;  // (stddev) Error of lidar measured distance
const double ERROR_LASER_ANGLE = 0.02;  // (stddev) Error of lidar measured angle

const double ERROR_PARTICLE_INITIAL_POS = 20;  // (stddev) Initial particle sampling position error
#else
// Using those values should give a very low positioning error
const double OFFSET_PARTICLE_RESAMPLE_POS = 0.0000001;
const double OFFSET_PARTICLE_RESAMPLE_ANGLE = 0.0000001;

const double ERROR_ESTIMATED_POS = 0.0000001;
const double ERROR_ESTIMATED_ANGLE = 0.0000001;

const double ERROR_LASER_ANGLE = 0.0000001;
const double ERROR_LASER_DISTANCE = 0.0000001;

const double ERROR_PARTICLE_INITIAL_POS = 0.0000001;
#endif

std::default_random_engine random_engine;  // Random engine used in the simulation
