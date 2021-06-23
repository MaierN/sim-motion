import pygame
import math
import numpy as np
import time

FPS = 60

def normalize(vec):
    return vec / np.linalg.norm(vec)

def move_towards(curr, target, max_dist):
    delta = target - curr
    if np.linalg.norm(delta) <= max_dist:
        return target
    return curr + normalize(delta) * max_dist

def rotation_matrix(angle):
    return np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])

class Robot:
    def __init__(self):
        self.pos = np.array([100.0, 100.0])
        self.angle = 0.0
        self.radius = 20.0
        self.speed = 300.0
        self.angular_speed = 25.0

        self.speeds_left = np.array([])
        self.speeds_right = np.array([])
        self.angles = np.array([])
    def direction(self):
        return np.array([math.cos(self.angle), math.sin(self.angle)])
    def orientation_pos(self):
        return self.pos + self.direction() * self.radius/2
    def left_wheel_pos(self):
        angle = self.angle - math.pi/2
        return self.pos + np.array([math.cos(angle), math.sin(angle)]) * self.radius
    def right_wheel_pos(self):
        angle = self.angle + math.pi/2
        return self.pos + np.array([math.cos(angle), math.sin(angle)]) * self.radius
    def size(self):
        return self.radius * 2

robot = Robot()
estimated_robot_1 = Robot()
estimated_robot_2 = Robot()
estimated_robot_r = Robot()
estimated_robot_t = Robot()
estimated_robot_s = Robot()

def draw_robot(robot, surface, color):
    pygame.draw.circle(surface, color, robot.pos, robot.radius, 1)
    pygame.draw.circle(surface, (0, 0, 0), robot.orientation_pos(), robot.radius/2)
    pygame.draw.circle(surface, (0, 255, 0), robot.left_wheel_pos(), robot.radius/8)
    pygame.draw.circle(surface, (0, 0, 255), robot.right_wheel_pos(), robot.radius/8)

def main():
    pygame.init()
    pygame.display.set_caption("motion simulation")

    surface = pygame.display.set_mode((500, 750))

    last_time = time.time()
    clock = pygame.time.Clock()
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        curr_time = time.time()
        time_delta = curr_time - last_time
        last_time = curr_time
        clock.tick(FPS)

        surface.fill((255, 255, 255))

        dt = 1 / FPS

        if pygame.mouse.get_pressed()[0]:
            old_left = robot.left_wheel_pos()
            old_right = robot.right_wheel_pos()

            old_angle = robot.angle

            iterations = 500
            for i in range(iterations):
                mouse_pos = np.array(pygame.mouse.get_pos())
                direction = move_towards(robot.direction(), normalize(mouse_pos - robot.pos), robot.angular_speed * (dt/iterations))

                robot.angle = math.atan2(direction[1], direction[0])
                robot.pos += direction * (dt/iterations) * robot.speed

            new_left = robot.left_wheel_pos()
            new_right = robot.right_wheel_pos()

            r_m = rotation_matrix(-robot.angle)
            old_left_rel = r_m @ (old_left - robot.pos)
            old_right_rel = r_m @ (old_right - robot.pos)

            d_left = np.linalg.norm(robot.left_wheel_pos() - old_left) * (1 if old_left_rel[0] <= 0 else -1)
            d_right = np.linalg.norm(robot.right_wheel_pos() - old_right) * (1 if old_right_rel[0] <= 0 else -1)

            #estimated_robot_1.angle = robot.angle
            #estimated_robot_2.angle = (old_angle + robot.angle) / 2






            robot.speeds_left = np.append(robot.speeds_left, d_left / dt)
            robot.speeds_right = np.append(robot.speeds_right, d_right / dt)
            avg_angle = (robot.angle + old_angle) / 2
            if abs(old_angle - robot.angle) > math.pi:
                if avg_angle > 0:
                    avg_angle -= math.pi
                else:
                    avg_angle += math.pi
            robot.angles = np.append(robot.angles, avg_angle)

        #robot.speeds_left = np.array([0, 0.0025, 0.0038, 0.0041, 0.0037, 0.0029, 0.0018])
        #robot.speeds_right = np.array([0, 0.0046, 0.0108, 0.0182, 0.0265, 0.0353, 0.0443])
        #robot.radius = 0.1
        #dt = 1

        n = len(robot.speeds_left)
        if n % 2 == 1 and n >= 3:
            speeds_center = (robot.speeds_left + robot.speeds_right) / 2
            omega = (robot.speeds_left - robot.speeds_right) / robot.size()

            def integrate_r(d):
                return np.append([0], d[0:n-1] * dt)

            def integrate_t(d):
                return np.append([0], ((d[0:n-1] + d[1:n]) / 2) * dt)

            def integrate_s(d):
                a_l = 2 * dt * (5/24 * np.append([0], d[0:n-1]) + 8/24 * d - 1/24 * np.append(d[1:n], [0]))
                a_r = 2 * dt * (-1/24 * np.append([0, 0], d[0:n-2]) + 8/24 * np.append([0], d[0:n-1]) + 5/24 * d)
                b_r = np.array([1, 0] * (n//2) + [1])
                b_l = np.array([0, 1] * (n//2) + [0])

                return a_l * b_l + a_r * b_r

            integrate = integrate_r
            theta = np.cumsum(integrate(omega))
            theta = robot.angles
            dx = speeds_center * np.cos(theta)
            dy = speeds_center * np.sin(theta)
            X = np.cumsum(integrate(dx))
            Y = np.cumsum(integrate(dy))
            estimated_robot_r.pos = np.array([X[-1] + 100, Y[-1] + 100])
            estimated_robot_r.angle = theta[-1]

            integrate = integrate_t
            theta = np.cumsum(integrate(omega))
            theta = robot.angles
            dx = speeds_center * np.cos(theta)
            dy = speeds_center * np.sin(theta)
            X = np.cumsum(integrate(dx))
            Y = np.cumsum(integrate(dy))
            estimated_robot_t.pos = np.array([X[-1] + 100, Y[-1] + 100])
            estimated_robot_t.angle = theta[-1]

            integrate = integrate_s
            theta = np.cumsum(integrate(omega))
            theta = robot.angles
            dx = speeds_center * np.cos(theta)
            dy = speeds_center * np.sin(theta)
            X = np.cumsum(integrate(dx))
            Y = np.cumsum(integrate(dy))
            estimated_robot_s.pos = np.array([X[-1] + 100, Y[-1] + 100])
            estimated_robot_s.angle = theta[-1]


        draw_robot(robot, surface, (255, 0, 0))
        draw_robot(estimated_robot_r, surface, (0, 255, 255))
        draw_robot(estimated_robot_t, surface, (255, 255, 0))
        draw_robot(estimated_robot_s, surface, (255, 0, 255))

        pygame.display.flip()


if __name__=="__main__":
    main()
