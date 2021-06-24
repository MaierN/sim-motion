#define HAVE_M_PI
#define _USE_MATH_DEFINES

#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>

#include "Config.h"
#include "Particle.hpp"
#include "Robot.hpp"
#include "Vector2.hpp"

static std::vector<double> integrate_cumul(std::vector<double> v, double dt, bool add_zero=false) {
    std::vector<double> res;
    double cumul = 0;
    //if (add_zero) res.push_back(0);
    for (size_t i = 0; i < v.size(); i++) {
        res.push_back(cumul);
        cumul += v[i] * dt;
    };
    res.push_back(cumul);
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
    //if (v1.size() != v2.size()) {
    //    std::cout << v1.size() << " " << v2.size() << std::endl;
    //    exit(1);
    //}
    for (size_t i = 0; i < v1.size(); i++) res.push_back(v1[i] * v2[i]);
    return res;
}

static void draw_circle(sf::RenderWindow& window, Vector2 center, double radius, const sf::Color& color,
                        bool outline = false) {
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
    sf::Vertex line[] = {sf::Vertex(sf::Vector2f((float)p1.x, (float)p1.y)),
                         sf::Vertex(sf::Vector2f((float)p2.x, (float)p2.y))};
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

static bool compare_particles(const Particle& p1, const Particle& p2) {
    return p1.weight < p2.weight;
}

int main(int argc, char** argv) {
    Robot robot;
    Robot estimated_robot;
    std::vector<Particle> particles;
    for (size_t i = 0; i < N_PARTICLES; i++) {
        particles.push_back(Particle());
    }
    Map map;
    std::vector<Vector2> sensor_points;

    sf::RenderWindow window(sf::VideoMode(SCREEN_WIDTH, SCREEN_HEIGHT), "sim motion");
    window.setFramerateLimit(FPS);

    sf::Font font;
    if (!font.loadFromFile("arial.ttf")) {
        std::cout << "font not found..." << std::endl;
        return 1;
    }
    sf::Text error_text;
    error_text.setFont(font);
    error_text.setCharacterSize(16);
    error_text.setFillColor(sf::Color(255, 0, 0));
    error_text.setPosition(20, 20);

    size_t f_count = 0;

    while (window.isOpen()) {
        f_count++;

        bool do_motion_update = true;
        bool do_sensor_update = f_count % 4 == 0;
        bool do_resampling = f_count % 4 == 0;

        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::Q) {
                    window.close();
                }
                if (event.key.code == sf::Keyboard::A) {
                    do_motion_update = true;
                }
                if (event.key.code == sf::Keyboard::S) {
                    do_sensor_update = true;
                }
                if (event.key.code == sf::Keyboard::D) {
                    do_resampling = true;
                }
                if (event.key.code == sf::Keyboard::F) {
                    do_motion_update = true;
                    do_sensor_update = true;
                    do_resampling = true;
                }
            }
        }

        window.clear(sf::Color::White);

        double dt = 1 / (double)FPS;

        {
            Vector2 old_left = robot.left_wheel_pos();
            Vector2 old_right = robot.right_wheel_pos();
            double old_angle = robot.angle;

            double d_left = 0;
            double d_right = 0;

            size_t iterations = 50;
            for (size_t i = 0; i < iterations; i++) {
                sf::Vector2i local_mouse_position = sf::Mouse::getPosition(window);
                Vector2 mouse_pos(local_mouse_position.x, local_mouse_position.y);
                Vector2 direction = robot.direction().moved_towards((mouse_pos - robot.pos).normalized(),
                                                                    robot.angular_speed * (dt / iterations));

                robot.angle = atan2(direction.y, direction.x);
                if (sf::Mouse::isButtonPressed(sf::Mouse::Left))
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

            d_left += std::normal_distribution<double>(0, 0.1)(random_engine);
            d_right += std::normal_distribution<double>(0, 0.1)(random_engine);
            avg_angle += std::normal_distribution<double>(0, 0.001)(random_engine);

            robot.speeds_center.push_back((d_left + d_right) / 2 / dt);
            robot.omega.push_back((d_left - d_right) / robot.size() / dt);
            robot.angles.push_back(avg_angle);
        }

        size_t n = robot.speeds_center.size();
        if (n % 2 == 1 && n >= 3) {
            std::vector<double> theta = integrate_cumul(robot.omega, dt, false);
            //theta = robot.angles;
            std::vector<double> dx = vec_mul(robot.speeds_center, vec_cos(theta));
            std::vector<double> dy = vec_mul(robot.speeds_center, vec_sin(theta));
            std::vector<double> estimated_xs = integrate_cumul(dx, dt);
            std::vector<double> estimated_ys = integrate_cumul(dy, dt);

            estimated_robot.pos = Vector2(100 + estimated_xs.back(), 100 + estimated_ys.back());
            estimated_robot.angle = theta.back();
        }

        draw_robot(window, robot, sf::Color(255, 0, 0), true);
        draw_robot(window, estimated_robot, sf::Color(255, 0, 255), true);

        for (const Line& obstacle : map.static_obstacles)
            draw_line(window, obstacle.p1, obstacle.p2, sf::Color(0, 255, 255));
        for (const Line& obstacle : map.dynamic_obstacles)
            draw_line(window, obstacle.p1, obstacle.p2, sf::Color(0, 255, 0));

        if (do_motion_update) {
            // std::cout << "motion update" << std::endl;
            Motion m = estimated_robot.get_motion_update();
            for (Particle& particle : particles) {
                particle.apply_motion(m);
                //particle.angle = robot.angle;
            }
        }

        if (do_sensor_update) {
            // std::cout << "sensor update" << std::endl;
            sensor_points.clear();
            for (size_t i = 0; i < N_LASER; i++) {
                LidarMeasure meas = robot.next_measure_lidar(map);
                sensor_points.push_back(meas.point);

                for (Particle& particle : particles) {
                    LidarMeasure p_meas = particle.sensor_measure(map, meas.angle);
                    double error = meas.distance - p_meas.distance;
                    double delta = 10;
                    if (error < -delta) error = 0;
                    double phi = 20;
                    double weight = exp(-(error * error) / (2 * phi * phi)) / (phi * sqrt(2 * M_PI));
                    particle.weight *= weight;
                }
            }

            double max_weight = 0;
            for (const Particle& particle : particles) {
                if (particle.weight >= max_weight) max_weight = particle.weight;
            }
            if (max_weight > 0) {
                for (Particle& particle : particles) {
                    particle.weight /= max_weight;
                }
            }
            std::sort(particles.begin(), particles.end(), compare_particles);
        }

        if (do_resampling) {
            // std::cout << "resampling" << std::endl;
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

                std::discrete_distribution<> distribution(weights.begin(), weights.end());
                std::vector<Particle> choices;
                for (size_t i = 0; i < particles.size(); i++) choices.push_back(particles[distribution(random_engine)]);

                particles.clear();
                double max_weight = 0;
                for (const Particle& particle : choices) {
                    if (particle.weight >= max_weight) max_weight = particle.weight;
                    particles.push_back(particle);
                }
                std::sort(particles.begin(), particles.end(), compare_particles);

                if (max_weight > 0) {
                    for (Particle& particle : particles) particle.weight /= max_weight;
                } else {
                    std::cout << "max_weight == 0..." << std::endl;
                }

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
                // std::cout << "Estimated pos: " << avg_pos << ", " << avg_angle << std::endl;
                double pos_error = (avg_pos - robot.pos).norm();
                double angle_error = abs(avg_angle - robot.angle);
                while (angle_error > M_PI) angle_error -= 2 * M_PI;
                angle_error = abs(angle_error);
                // std::cout << "Error: " << pos_error << ", " << angle_error << std::endl;
                error_text.setString("pos: " + (pos_error < 10 ? std::string("0") : std::string("")) +
                                     std::to_string(pos_error) + "\nang: " + std::to_string(angle_error));
            }
        }

        window.draw(error_text);

        for (const Particle& particle : particles) draw_particle(window, particle);
        for (const Vector2& point : sensor_points) draw_circle(window, point, 2, sf::Color(255, 0, 0));

        window.display();
    }

    return 0;
}
