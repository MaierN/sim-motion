#pragma once

#include <random>

//#define SANITY_CHECK

const int SCREEN_WIDTH = 500;
const int SCREEN_HEIGHT = 750;

const int FPS = 60;
const int N_LASER = 30;
const int N_PARTICLES = 400;
const int N_SELECTED_PARTICLES = 50;

const double ROBOT_LINEAR_SPEED = 800;
const double ROBOT_ANGULAR_SPEED = 30;
const bool ALWAYS_ROTATE = true;

const int UPDATE_DELAY = 4;

const double WEIGHT_DELTA = 10;
const double WEIGHT_PHI = 20;
const bool FILTER_MEASURES_TOO_CLOSE = false;
const bool FILTER_MEASURES_NOT_CLOSE_TO_WALL = true;
const double FILTER_MEASURES_CLOSE_TO_WALL_DISTANCE = 10;

const double LASER_RANGE = 10000;

const double OFFSET_PARTICLE_RESAMPLE_POS = 1;
const double OFFSET_PARTICLE_RESAMPLE_ANGLE = 0.005;

#ifndef SANITY_CHECK
    const double ERROR_ESTIMATED_POS = 0.5;
    const double ERROR_ESTIMATED_ANGLE = 0.01;

    const double ERROR_LASER_ANGLE = 0.01;
    const double ERROR_LASER_DISTANCE = 2;

    const double ERROR_PARTICLE_INITIAL_POS = 20;
#else
    const double ERROR_ESTIMATED_POS = 0.0000001;
    const double ERROR_ESTIMATED_ANGLE = 0.0000001;

    const double ERROR_LASER_ANGLE = 0.0000001;
    const double ERROR_LASER_DISTANCE = 0.0000001;

    const double ERROR_PARTICLE_INITIAL_POS = 0.0000001;
#endif

std::default_random_engine random_engine;
