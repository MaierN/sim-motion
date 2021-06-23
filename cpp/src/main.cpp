#define HAVE_M_PI
#define _USE_MATH_DEFINES

#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>

#include "Config.h"
#include "Particle.hpp"
#include "Robot.hpp"
#include "Vector2.hpp"

static std::vector<double> integrate_cumul(std::vector<double> v, double dt) {
    std::vector<double> res;
    double cumul = 0;
    for (size_t i = 0; i < v.size(); i++) {
        res.push_back(cumul);
        cumul += v[i] * dt;
    };
    return res;
}

static std::vector<double> vec_sin(std::vector<double> v) {
    std::vector<double> res;
    for (size_t i = 0; i < v.size(); i++) res.push_back(sin(v[i]));
    return res;
}

static std::vector<double> vec_cos(std::vector<double> v) {
    std::vector<double> res;
    for (size_t i = 0; i < v.size(); i++) res.push_back(cos(v[i]));
    return res;
}

static std::vector<double> vec_mul(std::vector<double> v1, std::vector<double> v2) {
    std::vector<double> res;
    for (size_t i = 0; i < v1.size(); i++) res.push_back(v1[i] * v2[i]);
    return res;
}

int main(int argc, char** argv) {
    Robot robot;
    Robot estimated_robot;
    std::vector<Particle> particles;
    for (size_t i = 0; i < N_PARTICLES; i++) {
        particles.push_back(Particle());
    }

    //SDL_Window* window = NULL;
    //SDL_Surface* surface = NULL;

    //SDL_Init(SDL_INIT_VIDEO);

    //window = SDL_CreateWindow("sim motion", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH,
    //                          SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    //surface = SDL_GetWindowSurface(window);

    bool running = true;
    while (running) {
        bool do_motion_update = true;
        bool do_sensor_update = false;
        bool do_resampling = false;

        //SDL_Event event;
        /*while (SDL_PollEvent(&event)) {
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
        }*/

        //SDL_FillRect(surface, NULL, SDL_MapRGB(surface->format, 255, 255, 255));

        double dt = 1 / FPS;

        int mouse_x, mouse_y;
        //uint32_t mouse_buttons = SDL_GetMouseState(&mouse_x, &mouse_y);
        /*if (mouse_buttons & SDL_BUTTON(1)) {
            Vector2 old_left = robot.left_wheel_pos();
            Vector2 old_right = robot.right_wheel_pos();
            double old_angle = robot.angle;

            double d_left = 0;
            double d_right = 0;

            size_t iterations = 50;
            for (size_t i = 0; i < iterations; i++) {
                Vector2 mouse_pos(mouse_x, mouse_y);
                Vector2 direction = robot.direction().moved_towards((mouse_pos - robot.pos).normalized(),
                                                                    robot.angular_speed * (dt / iterations));

                robot.angle = atan2(direction.y, direction.x);
                robot.pos += direction * (dt / iterations) * robot.linear_speed;

                Vector2 new_left = robot.left_wheel_pos();
                Vector2 new_right = robot.right_wheel_pos();

                Vector2 old_left_rel = (old_left - robot.pos).rotated(-robot.angle);
                Vector2 old_right_rel = (old_right - robot.pos).rotated(-robot.angle);

                d_left += (new_left - old_left).norm() * (old_left_rel.x <= 0 ? 1 : -1);
                d_right += (new_right - old_right).norm() * (old_right_rel.x <= 0 ? 1 : -1);

                old_left = new_left;
                old_right = new_right;
            }

            double avg_angle = (robot.angle + old_angle) / 2;
            if (abs(old_angle - robot.angle) > M_PI) {
                if (avg_angle > 0)
                    avg_angle -= M_PI;
                else
                    avg_angle += M_PI;
            }

            robot.speeds_center.push_back((d_left + d_right) / 2 / dt);
            robot.omega.push_back((d_left - d_right) / robot.size() / dt);
            robot.angles.push_back(avg_angle);
        }*/

        size_t n = robot.speeds_center.size();
        if (n % 2 == 1 && n >= 3) {
            std::vector<double> theta = integrate_cumul(robot.omega, dt);
            std::vector<double> dx = vec_mul(robot.speeds_center, vec_cos(theta));
            std::vector<double> dy = vec_mul(robot.speeds_center, vec_sin(theta));
            std::vector<double> estimated_xs = integrate_cumul(dx, dt);
            std::vector<double> estimated_ys = integrate_cumul(dy, dt);

            estimated_robot.pos = Vector2(100 + estimated_xs.back(), 100 + estimated_ys.back());
            estimated_robot.angle = theta.back();
        }

        //SDL_UpdateWindowSurface(window);
    }

    //SDL_DestroyWindow(window);
    //SDL_Quit();

    return 0;
}
