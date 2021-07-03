#define HAVE_M_PI
#define _USE_MATH_DEFINES

#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>

#include "Config.h"
#include "Particle.hpp"
#include "Robot.hpp"
#include "Vector2.hpp"

static void draw_circle(sf::RenderWindow& window, Vector2 center, double radius, const sf::Color& color, bool outline = false) {
    sf::CircleShape mainCircle((float)radius);
    mainCircle.setPosition(sf::Vector2f((float)(center.x - radius), (float)(center.y - radius)));
    if (outline) {
        mainCircle.setOutlineColor(color);
        mainCircle.setOutlineThickness(2);
    } else {
        mainCircle.setFillColor(color);
    }
    window.draw(mainCircle);
}
static void draw_line(sf::RenderWindow& window, Vector2 p1, Vector2 p2, const sf::Color& color) {
    sf::Vertex line[] = {sf::Vertex(sf::Vector2f((float)p1.x, (float)p1.y)), sf::Vertex(sf::Vector2f((float)p2.x, (float)p2.y))};
    line[0].color = color;
    line[1].color = color;
    window.draw(line, 2, sf::Lines);
}
static void draw_robot(sf::RenderWindow& window, const Robot& robot, const sf::Color& color, bool outline) {
    draw_circle(window, robot.pos, robot.radius, color, outline);
    draw_circle(window, robot.orientation_pos(), robot.radius / 2, color);
    draw_circle(window, robot.left_wheel_pos(), robot.radius / 8, sf::Color(0, 255, 0));
    draw_circle(window, robot.right_wheel_pos(), robot.radius / 8, sf::Color(0, 0, 255));
}
static void draw_particle(sf::RenderWindow& window, const Particle& particle) {
    sf::Color color((uint8_t)(255 * particle.weight), 0, (uint8_t)(255 * particle.weight));
    draw_circle(window, particle.pos, 2, color);
    Vector2 direction(cos(particle.angle), sin(particle.angle));
    draw_line(window, particle.pos, particle.pos + direction * 7, color);
}

int main(int argc, char** argv) {
    Robot robot;            // "Physical" robot
    Robot estimated_robot;  // Estimation based on odometry
    Robot corrected_robot;  // Estimation based on odometry and lidar Monte Carlo simulation

    std::vector<Particle> particles;  // Potential robot positions
    for (size_t i = 0; i < N_PARTICLES; i++) {
        particles.push_back(Particle());
    }

    Map map;                                      // Game table
    std::vector<Vector2> sensor_points;           // Points seen with the lidar that are close enough to a wall
    std::vector<Vector2> sensor_points_rejected;  // Points seen with the lidar that are too far from the walls

    sf::RenderWindow window(sf::VideoMode(SCREEN_WIDTH, SCREEN_HEIGHT), "sim motion");
    window.setFramerateLimit(FPS);

    sf::Font font;
    if (!font.loadFromFile("arial.ttf")) {
        std::cout << "font not found..." << std::endl;
        return 1;
    }
    sf::Text error_text;  // Text to show the current position error
    error_text.setFont(font);
    error_text.setCharacterSize(16);
    error_text.setFillColor(sf::Color(255, 0, 0));
    error_text.setPosition(20, 20);

    size_t f_count = 0;  // Frames counter

    // Main simulation loop
    while (window.isOpen()) {
        f_count++;

        bool do_motion_update = true;
        bool do_sensor_update = f_count % UPDATE_DELAY == 0;
        bool do_resampling = f_count % UPDATE_DELAY == 0;

        // Press 'q' or click the "close window" button to end the simulation
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::Q) {
                    window.close();
                }
            }
        }

        window.clear(sf::Color::White);

        double dt = 1 / (double)FPS;  // Time elapsed since the last update

        if (ALWAYS_ROTATE || sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
            // Update physical robot angle based on cursor position
            sf::Vector2i local_mouse_position = sf::Mouse::getPosition(window);
            Vector2 mouse_pos(local_mouse_position.x, local_mouse_position.y);
            Vector2 direction = robot.direction().moved_towards((mouse_pos - robot.pos).normalized(), robot.angular_speed * dt);
            robot.angle = atan2(direction.y, direction.x);

            // Move physical robot forward based on mouse click
            if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) robot.pos += direction * dt * robot.linear_speed;

            // Replicate motion on estimated robot with a random error
            Motion m = robot.get_motion_update();
            estimated_robot.pos +=
                (m.pos_diff + Vector2(std::normal_distribution<double>(0, ERROR_ESTIMATED_POS)(random_engine), std::normal_distribution<double>(0, ERROR_ESTIMATED_POS)(random_engine)))
                    .rotated(estimated_robot.angle);
            estimated_robot.angle += m.angle_diff + std::normal_distribution<double>(0, ERROR_ESTIMATED_ANGLE)(random_engine);
        }

        draw_robot(window, robot, sf::Color(255, 0, 0), true);
        draw_robot(window, estimated_robot, sf::Color(255, 0, 255), true);

        for (const Line& obstacle : map.static_obstacles) draw_line(window, obstacle.p1, obstacle.p2, sf::Color(0, 255, 255));
        for (const Line& obstacle : map.dynamic_obstacles) draw_line(window, obstacle.p1, obstacle.p2, sf::Color(0, 255, 0));

        if (do_motion_update) {
            // Motion update: move every particles based on odometry
            Motion m = estimated_robot.get_motion_update();
            for (Particle& particle : particles) {
                particle.apply_motion(m);
            }
            corrected_robot.pos += m.pos_diff.rotated(corrected_robot.angle);
            corrected_robot.angle += m.angle_diff;
        }

        if (do_sensor_update) {
            // Sensor update: update particle weights based on lidar measure
            sensor_points.clear();
            sensor_points_rejected.clear();
            for (size_t i = 0; i < N_LASER; i++) {
                // Take one measure
                LidarMeasure meas = robot.next_measure_lidar(map);
                Vector2 corrected_point = corrected_robot.pos + Vector2(cos(meas.angle + corrected_robot.angle), sin(meas.angle + corrected_robot.angle)) * meas.distance;
                // Reject measure if not plausible (too far from walls) to avoid interferences from dynamic obstacles
                double min_dist_static_obstacle = map.min_dist_static_obstacle(corrected_point);
                if (FILTER_MEASURES_NOT_CLOSE_TO_WALL && min_dist_static_obstacle > FILTER_MEASURES_CLOSE_TO_WALL_DISTANCE) {
                    sensor_points_rejected.push_back(corrected_point);
                    continue;
                } else {
                    sensor_points.push_back(corrected_point);
                }

                // Update every particle weight (if the particle would have given a very close measure -> higher weight)
                for (Particle& particle : particles) {
                    LidarMeasure p_meas = particle.sensor_measure(map, meas.angle);
                    double error = meas.distance - p_meas.distance;
                    if (FILTER_MEASURES_TOO_CLOSE && error < -WEIGHT_DELTA) error = 0;
                    double weight = exp(-(error * error) / (2 * WEIGHT_PHI * WEIGHT_PHI)) / (WEIGHT_PHI * sqrt(2 * M_PI));
                    particle.weight *= weight;
                }
            }

            // Normalize particle weights
            double max_weight = 0;
            for (const Particle& particle : particles) {
                if (particle.weight >= max_weight) max_weight = particle.weight;
            }
            if (max_weight > 0) {
                for (Particle& particle : particles) {
                    particle.weight /= max_weight;
                }
            }
            std::sort(particles.begin(), particles.end(), [](const Particle& p1, const Particle& p2) -> bool { return p1.weight < p2.weight; });
        }

        if (do_resampling) {
            // Resampling: create a new cloud of particles by sampling the existing ones based on their weight
            double weight_sum = 0;
            for (const Particle& particle : particles) weight_sum += particle.weight;
            if (weight_sum == 0) {
                std::cout << "weight_sum == 0..." << std::endl;
                for (Particle& particle : particles) particle.weight = 1;
                weight_sum = (double)particles.size();
            } else {
                std::vector<double> weights;
                for (Particle& particle : particles) {
                    particle.weight /= weight_sum;
                    weights.push_back(particle.weight);
                }

                // Sample new particles
                std::discrete_distribution<> distribution(weights.begin(), weights.end());
                std::vector<Particle> choices;
                for (size_t i = 0; i < particles.size(); i++) choices.push_back(particles[distribution(random_engine)]);

                // Replace old particles with new ones
                particles.clear();
                double max_weight = 0;
                for (const Particle& particle : choices) {
                    if (particle.weight >= max_weight) max_weight = particle.weight;
                    particles.push_back(particle);
                }
                std::sort(particles.begin(), particles.end(), [](const Particle& p1, const Particle& p2) -> bool { return p1.weight < p2.weight; });

                if (max_weight > 0) {
                    for (Particle& particle : particles) particle.weight /= max_weight;
                } else {
                    std::cout << "max_weight == 0..." << std::endl;
                }

                /*
                // Method 1) Compute corrected robot estimation based on weighted average of particles
                double selected_weight = 0;
                Vector2 avg_pos(0, 0);
                double avg_angle = 0;
                for (size_t i = 0; i < N_SELECTED_PARTICLES; i++) {
                    const Particle& p = particles[particles.size() - 1 - i];
                    avg_pos += p.pos * p.weight;
                    avg_angle += p.angle * p.weight;
                    selected_weight += p.weight;
                }
                avg_pos /= selected_weight;
                avg_angle /= selected_weight;
                */

                std::vector<Particle> selected_particles_pos_x;
                std::vector<Particle> selected_particles_pos_y;
                std::vector<Particle> selected_particles_angle;
                double total_selected_weight = 0;
                for (size_t i = 0; i < N_SELECTED_PARTICLES; i++) {
                    const Particle& p = particles[particles.size() - 1 - i];
                    selected_particles_pos_x.push_back(p);
                    selected_particles_pos_y.push_back(p);
                    selected_particles_angle.push_back(p);
                    total_selected_weight += p.weight;
                }
                std::sort(selected_particles_pos_x.begin(), selected_particles_pos_x.end(), [](const Particle& p1, const Particle& p2) -> bool { return p1.pos.x < p2.pos.x; });
                std::sort(selected_particles_pos_y.begin(), selected_particles_pos_y.end(), [](const Particle& p1, const Particle& p2) -> bool { return p1.pos.y < p2.pos.y; });
                std::sort(selected_particles_angle.begin(), selected_particles_angle.end(), [](const Particle& p1, const Particle& p2) -> bool { return p1.angle < p2.angle; });
                /*
                // Method 2) Compute corrected robot estimation based on weighted median of particles
                double median_pos_x = 0;
                double median_pos_y = 0;
                double median_angle = 0;
                double accumulator = 0;
                for (size_t i = 0; i < selected_particles_pos_x.size(); i++) {
                    accumulator += selected_particles_pos_x[i].weight;
                    if (accumulator >= total_selected_weight / 2) {
                        median_pos_x = selected_particles_pos_x[i].pos.x;
                        break;
                    }
                }
                accumulator = 0;
                for (size_t i = 0; i < selected_particles_pos_y.size(); i++) {
                    accumulator += selected_particles_pos_y[i].weight;
                    if (accumulator >= total_selected_weight / 2) {
                        median_pos_y = selected_particles_pos_y[i].pos.y;
                        break;
                    }
                }
                accumulator = 0;
                for (size_t i = 0; i < selected_particles_angle.size(); i++) {
                    accumulator += selected_particles_angle[i].weight;
                    if (accumulator >= total_selected_weight / 2) {
                        median_angle = selected_particles_angle[i].angle;
                        break;
                    }
                }
                Vector2 avg_pos(median_pos_x, median_pos_y);
                double avg_angle = median_angle;
                */
                // Method 3) Compute corrected robot estimation based on median of particles without taking weights into
                // account
                Vector2 avg_pos(selected_particles_pos_x[selected_particles_pos_x.size() / 2].pos.x, selected_particles_pos_y[selected_particles_pos_y.size() / 2].pos.y);
                double avg_angle = selected_particles_angle[selected_particles_angle.size() / 2].angle;

                corrected_robot.pos = avg_pos;
                corrected_robot.angle = avg_angle;

                // Show error on screen
                double pos_error = (avg_pos - robot.pos).norm();
                double angle_error = abs(avg_angle - robot.angle);
                while (angle_error > M_PI) angle_error -= 2 * M_PI;
                angle_error = abs(angle_error);
                error_text.setString("pos: " + (pos_error < 10 ? std::string("0") : std::string("")) + std::to_string(pos_error) + "\nang: " + std::to_string(angle_error));
            }
        }

        window.draw(error_text);

        draw_robot(window, corrected_robot, sf::Color(0, 255, 0), true);
        for (const Particle& particle : particles) draw_particle(window, particle);
        for (const Vector2& point : sensor_points) draw_circle(window, point, 2, sf::Color(255, 0, 0));
        for (const Vector2& point : sensor_points_rejected) draw_circle(window, point, 2, sf::Color(255, 0, 255));

        window.display();
    }

    return 0;
}
