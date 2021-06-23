import pygame
import math
import numpy as np
import time

FPS = 60
N_LASER = 10
N_PARTICLES = 500
N_SELECTED_PARTICLES = 5

static_obstacles = np.array([
    [
        [10, 10],
        [490, 10],
    ],
    [
        [490, 10],
        [490, 740],
    ],
    [
        [490, 740],
        [10, 740],
    ],
    [
        [10, 740],
        [10, 10],
    ],
    [
        [200, 200],
        [200, 500],
    ],
])
dynamic_obstacles = np.array([
    [
        [400, 400],
        [300, 500],
    ],
])

def normalize(vec):
    return vec / np.linalg.norm(vec)

def move_towards(curr, target, max_dist):
    delta = target - curr
    if np.linalg.norm(delta) <= max_dist:
        return target
    return curr + normalize(delta) * max_dist

def rotation_matrix(angle):
    return np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])

def line_line_intersection(l1, l2):
    x1 = l1[0][0]
    y1 = l1[0][1]
    x2 = l1[1][0]
    y2 = l1[1][1]
    x3 = l2[0][0]
    y3 = l2[0][1]
    x4 = l2[1][0]
    y4 = l2[1][1]
    det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(det) <= 1e-3:
        return None
    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / det
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / det
    return np.array([px, py])

def point_in_segment(s, p):
    direction = s[1] - s[0]
    angle = math.atan2(direction[1], direction[0])
    r_m = rotation_matrix(-angle)
    s_0 = r_m @ s[0]
    s_1 = r_m @ s[1]
    p = r_m @ p
    return (p[0] >= s_0[0] and p[0] <= s_1[0]) or (p[0] <= s_0[0] and p[0] >= s_1[0])

def segment_segment_intersection(s1, s2):
    p = line_line_intersection(s1, s2)
    if p is None:
        return None
    return p if point_in_segment(s1, p) and point_in_segment(s2, p) else None

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

        self.next_lidar_angle = 0.0

        self.last_pos = self.pos
        self.last_angle = self.angle
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
    def next_measure_lidar(self, surface):
        laser_angle = self.next_lidar_angle + self.angle + np.random.normal(scale=0.01)
        direction = np.array([math.cos(laser_angle), math.sin(laser_angle)])
        laser_range = 10000
        laser_end_point = self.pos + direction * laser_range
        laser_segment = np.array([self.pos, laser_end_point])
        #pygame.draw.line(surface, (255, 255, 0), laser_segment[0], laser_segment[1])
        min_dist = laser_range
        min_point = laser_end_point
        for obstacle in static_obstacles:
            point = segment_segment_intersection(laser_segment, obstacle)
            if point is not None:
                distance = np.linalg.norm(robot.pos - point)
                if distance <= min_dist:
                    min_dist = distance
                    min_point = point + np.random.normal(scale=2, size=2)
        for obstacle in dynamic_obstacles:
            point = segment_segment_intersection(laser_segment, obstacle)
            if point is not None:
                distance = np.linalg.norm(robot.pos - point)
                if distance <= min_dist:
                    min_dist = distance
                    min_point = point + np.random.normal(scale=2, size=2)
        old_lidar_angle = self.next_lidar_angle
        self.next_lidar_angle += 2*math.pi/N_LASER
        return old_lidar_angle, min_dist, min_point
    def get_motion_update(self):
        m_r = rotation_matrix(-self.last_angle)
        pos_diff = m_r @ (self.pos - self.last_pos)
        angle_diff = self.angle - self.last_angle

        self.last_pos = self.pos
        self.last_angle = self.angle

        return pos_diff, angle_diff

class Particle:
    def __init__(self):
        self.pos = np.random.uniform(low=[0.0, 0.0], high=[500.0, 750.0], size=2)
        self.pos = np.array([100, 100]) + np.random.normal(scale=20, size=2)
        self.angle = np.random.uniform(low=0.0, high=math.pi*2)
        self.weight = 1
    @classmethod
    def copy_of(cls, particle, noise=True):
        p = Particle()
        p.pos = np.copy(particle.pos)
        p.angle = particle.angle
        p.weight = particle.weight
        p.pos += np.random.normal(scale=2, size=2)
        p.angle += np.random.normal(scale=0.05)
        return p
    def apply_motion(self, pos_diff, angle_diff):
        m_r = rotation_matrix(self.angle)
        self.pos += m_r @ pos_diff
        self.angle = (self.angle + angle_diff) % (2 * math.pi)
    def sensor_measure(self, angle):
        laser_angle = angle + self.angle
        direction = np.array([math.cos(laser_angle), math.sin(laser_angle)])
        laser_range = 10000
        laser_end_point = self.pos + direction * laser_range
        laser_segment = np.array([self.pos, laser_end_point])
        min_dist = laser_range
        min_point = laser_end_point
        for dynamic_obstacle in static_obstacles:
            point = segment_segment_intersection(laser_segment, dynamic_obstacle)
            if point is not None:
                distance = np.linalg.norm(robot.pos - point)
                if distance <= min_dist:
                    min_dist = distance
                    min_point = point
        return angle, min_dist, min_point

robot = Robot()
estimated_robot = Robot()

def draw_robot(robot, surface, color, outline=1):
    pygame.draw.circle(surface, color, robot.pos, robot.radius, outline)
    pygame.draw.circle(surface, (0, 0, 0), robot.orientation_pos(), robot.radius/2)
    pygame.draw.circle(surface, (0, 255, 0), robot.left_wheel_pos(), robot.radius/8)
    pygame.draw.circle(surface, (0, 0, 255), robot.right_wheel_pos(), robot.radius/8)

def draw_particle(particle, surface, color):
    color = np.array(color) * particle.weight
    pygame.draw.circle(surface, color, particle.pos, 2)
    direction = np.array([math.cos(particle.angle), math.sin(particle.angle)])
    pygame.draw.line(surface, color, particle.pos, particle.pos + direction * 7)

def main():
    particles = []
    for i in range(N_PARTICLES):
        particles.append(Particle())
    sensor_points = []

    pygame.init()
    pygame.display.set_caption("motion simulation")

    surface = pygame.display.set_mode((500, 750))

    last_time = time.time()
    clock = pygame.time.Clock()
    running = True

    while running:
        do_motion_update = True
        do_sensor_update = False
        do_resampling = False
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == 27:
                    running = False
                if event.key == ord('a'):
                    do_motion_update = True
                if event.key == ord('s'):
                    do_sensor_update = True
                if event.key == ord('d'):
                    do_resampling = True
                if event.key == ord('f'):
                    do_motion_update = True
                    do_sensor_update = True
                    do_resampling = True
        curr_time = time.time()
        time_delta = curr_time - last_time
        last_time = curr_time
        clock.tick(FPS)

        surface.fill((255, 255, 255))

        dt = 1 / FPS

        if pygame.mouse.get_pressed()[0]:
            old_left = robot.left_wheel_pos()
            old_right = robot.right_wheel_pos()
            d_left = 0.0
            d_right = 0.0

            old_angle = robot.angle

            iterations = 50
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

                d_left += np.linalg.norm(robot.left_wheel_pos() - old_left) * (1 if old_left_rel[0] <= 0 else -1)
                d_right += np.linalg.norm(robot.right_wheel_pos() - old_right) * (1 if old_right_rel[0] <= 0 else -1)

                old_left = new_left
                old_right = new_right

            avg_angle = (robot.angle + old_angle) / 2
            if abs(old_angle - robot.angle) > math.pi:
                if avg_angle > 0:
                    avg_angle -= math.pi
                else:
                    avg_angle += math.pi

            robot.speeds_left = np.append(robot.speeds_left, d_left / dt)
            robot.speeds_right = np.append(robot.speeds_right, d_right / dt)

            robot.angles = np.append(robot.angles, avg_angle)

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

            integrate = integrate_s
            theta = np.cumsum(integrate(omega))
            #theta = robot.angles
            dx = speeds_center * np.cos(theta)
            dy = speeds_center * np.sin(theta)
            X = np.cumsum(integrate(dx))
            Y = np.cumsum(integrate(dy))
            estimated_robot.pos = np.array([X[-1] + 100, Y[-1] + 100])
            estimated_robot.angle = theta[-1]

        draw_robot(robot, surface, (255, 0, 0), 0)
        draw_robot(estimated_robot, surface, (255, 0, 255))

        for static_obstacle in static_obstacles:
            pygame.draw.line(surface, (0, 255, 255), static_obstacle[0], static_obstacle[1])
        for dynamic_obstacle in dynamic_obstacles:
            pygame.draw.line(surface, (0, 255, 0), dynamic_obstacle[0], dynamic_obstacle[1])

        if do_motion_update:
            #print('motion update')
            pos_diff, angle_diff = estimated_robot.get_motion_update()
            for particle in particles:
                particle.apply_motion(pos_diff, angle_diff)

        if do_sensor_update:
            print('sensor update')
            sensor_points.clear()
            for i in range(N_LASER):
                angle, distance, point = robot.next_measure_lidar(surface)
                sensor_points.append(point)

                for particle in particles:
                    p_angle, p_distance, p_point = particle.sensor_measure(angle)
                    error = distance - p_distance
                    delta = 5
                    weight = 1
                    if error < -delta:
                        error = 0
                    phi = 10
                    weight = math.exp(-(error * error) / (2*phi*phi)) / (phi * math.sqrt(2 * math.pi))
                    particle.weight *= weight

            max_weight = 0
            for particle in particles:
                if particle.weight >= max_weight:
                    max_weight = particle.weight
            if max_weight > 0:
                for particle in particles:
                    particle.weight /= max_weight
            particles = sorted(particles, key=lambda p: p.weight)

        if do_resampling:
            print('resampling')
            weight_sum = sum([particle.weight for particle in particles])
            if weight_sum == 0:
                print('weight_sum == 0...')
                for particle in particles:
                    particle.weight = 1
                weight_sum = sum([particle.weight for particle in particles])
            for particle in particles:
                particle.weight /= weight_sum
            choices = np.random.choice(particles, size=len(particles), replace=True, p=[particle.weight for particle in particles])
            particles.clear()
            particles = sorted([Particle.copy_of(particle) for particle in choices], key=lambda p: p.weight)
            max_weight = 0
            for particle in particles:
                if particle.weight >= max_weight:
                    max_weight = particle.weight
            if max_weight > 0:
                for particle in particles:
                    particle.weight /= max_weight
            else:
                print('no......')
            selected = particles[-N_SELECTED_PARTICLES-1:-1]
            selected_weight = 0.0
            avg_pos = np.array([0.0, 0.0])
            avg_angle = 0.0
            for p in selected:
                avg_pos += p.pos * p.weight
                avg_angle += p.angle * p.weight
                selected_weight += p.weight
            avg_pos /= selected_weight
            avg_angle /= selected_weight
            print('Estimated pos: ' + str(avg_pos) + ', ' + str(avg_angle))
            print('Error: ' + str(np.linalg.norm(avg_pos - robot.pos)))

        for particle in particles:
            draw_particle(particle, surface, (255, 0, 255))

        for point in sensor_points:
            pygame.draw.circle(surface, (255, 0, 0), point, 2)

        pygame.display.flip()

if __name__=="__main__":
    main()
