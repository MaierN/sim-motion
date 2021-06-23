#include <iostream>

#include "SDL.h"
#include "Vector2.hpp"

const int FPS = 60;
const int N_LASER = 10;
const int N_PARTICLES = 500;
const int N_SELECTED_PARTICLES = 5;

const int SCREEN_WIDTH = 500;
const int SCREEN_HEIGHT = 750;

int main(int argc, char** argv) {
    std::cout << "go..." << std::endl;

    SDL_Window* window = NULL;
    SDL_Surface* surface = NULL;

    std::cout << "f" << std::endl;
    SDL_Init(SDL_INIT_VIDEO);
    std::cout << "a" << std::endl;

    window = SDL_CreateWindow("sim motion", SDL_WINDOWPOS_UNDEFINED,
                              SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH,
                              SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    surface = SDL_GetWindowSurface(window);

    bool running = true;
    while (running) {
        bool do_motion_update = true;
        bool do_sensor_update = false;
        bool do_resampling = false;

        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            }
            if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.scancode == SDL_SCANCODE_Q) {
                    running = false;
                }
                if (event.key.keysym.scancode == SDL_SCANCODE_A) {
                    do_motion_update = true;

                    Vector2 a(3, 4);
                    Vector2 b(5, 6);

                    std::cout << a << " + " << b << " = " << (a + b)
                              << std::endl;
                }
                if (event.key.keysym.scancode == SDL_SCANCODE_S) {
                    do_sensor_update = true;
                }
                if (event.key.keysym.scancode == SDL_SCANCODE_D) {
                    do_resampling = true;
                }
                if (event.key.keysym.scancode == SDL_SCANCODE_F) {
                    do_motion_update = true;
                    do_sensor_update = true;
                    do_resampling = true;
                }
            }
        }
    }

    SDL_FillRect(surface, NULL, SDL_MapRGB(surface->format, 255, 255, 255));

    SDL_UpdateWindowSurface(window);
    // SDL_Delay(2000);

    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
