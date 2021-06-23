#include <SDL.h>

#include <iostream>

#include "Config.h"
#include "Particle.hpp"
#include "Robot.hpp"
#include "Vector2.hpp"

int main(int argc, char** argv) {
    Robot robot;
    Robot estimated_robot;
    std::vector<Particle> particles;
    for (size_t i = 0; i < N_PARTICLES; i++) {
        particles.push_back(Particle());
    }

    SDL_Window* window = NULL;
    SDL_Surface* surface = NULL;

    SDL_Init(SDL_INIT_VIDEO);

    window = SDL_CreateWindow("sim motion", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH,
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

                    std::cout << a << " + " << b << " = " << (a + b) << std::endl;
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

        double dt = 1 / FPS;

        int mouse_x, mouse_y;
        uint32_t mouse_buttons = SDL_GetMouseState(&mouse_x, &mouse_y);
        if (mouse_buttons & SDL_BUTTON(1)) {
            std::cout << "a" << std::endl;
        }
    }

    SDL_FillRect(surface, NULL, SDL_MapRGB(surface->format, 255, 255, 255));

    SDL_UpdateWindowSurface(window);
    // SDL_Delay(2000);

    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
