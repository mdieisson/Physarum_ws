# Atualizacoes no arquivo environment.py para suportar atenuacao individual do sinal de cada robo

import numpy as np
from scipy.ndimage import gaussian_filter

class Environment:
    def __init__(self, width, height, decay_factor=0.95, diffusion_sigma=1.0):
        self.width = width
        self.height = height
        self.decay_factor = decay_factor
        self.diffusion_sigma = diffusion_sigma

        self.trail_map = np.zeros((height, width), dtype=np.float32)
        self.stable_trail_map = np.zeros((height, width), dtype=np.float32)
        self.mass_map = np.zeros((height, width), dtype=np.float32)
        self.food_map = np.zeros((height, width), dtype=np.float32)
        self.food_sources = []
        self.obstacle_sources = []
        self.max_food_intensity = 4
        self.obstacle_map = np.zeros((self.height, self.width), dtype=bool)
        self.repellent_map = np.zeros((height, width), dtype=np.float32)
        self.food_maps_by_robot = {}  # novo: comida individual por robo

    def is_free(self, x, y):
        return 0 <= x < self.height and 0 <= y < self.width and self.obstacle_map[x, y] == 0

    def add_obstacle(self, y, x, size=2, intensity=-20.0):
        if (y, x) not in self.obstacle_sources:
            self.obstacle_sources.append((y, x))
        yy, xx = np.ogrid[:self.height, :self.width]
        dist2 = (yy - y) ** 2 + (xx - x) ** 2
        sigma = size / 2.0
        gaussian = intensity * np.exp(-dist2 / (2 * sigma**2))
        self.repellent_map += gaussian
        
        self.obstacle_map[yy, xx] |= dist2 <= size**2  # marca obstaculo como True



    def add_robot_signal(self, robot_id, y, x, intensity=2.0, spread=20.0):
        if robot_id not in self.food_maps_by_robot:
            self.food_maps_by_robot[robot_id] = np.zeros((self.height, self.width), dtype=np.float32)
            

        y_grid, x_grid = np.ogrid[-y:self.height - y, -x:self.width - x]
        gaussian = intensity * np.exp(-(x_grid**2 + y_grid**2) / (2 * spread**2))
        attenuation_map = np.where(self.obstacle_map, 0.6, 0.9)  # 30% de intensidade em containers
        gaussian *= attenuation_map

        self.food_maps_by_robot[robot_id] = np.maximum(self.food_maps_by_robot[robot_id], gaussian)

    def update_food(self):
        self.food_map.fill(0)
        for robot_map in self.food_maps_by_robot.values():
            blurred = gaussian_filter(robot_map, sigma=1.0)
            self.food_map += blurred * 0.99

    def sense(self, x, y):
        if 0 <= int(x) < self.height and 0 <= int(y) < self.width:
            mass = self.mass_map[int(x), int(y)]
            trail = self.trail_map[int(x), int(y)]
            food = self.food_map[int(x), int(y)]
            return mass, trail, food
        else:
            return 0.0, 0.0, 0.0

    def reset_mass(self):
        self.mass_map.fill(0)

    def deposit_mass(self, x, y, amount=1.0):
        if 0 <= int(x) < self.height and 0 <= int(y) < self.width:
            self.mass_map[int(x), int(y)] += amount

    def deposit_trail(self, x, y, amount=1.0):
        if 0 <= int(x) < self.height and 0 <= int(y) < self.width:
            self.trail_map[int(x), int(y)] += amount
            self.stable_trail_map[int(x), int(y)] += amount * 0.5

    def update_trail(self):
        self.trail_map *= self.decay_factor
       # self.trail_map += self.repellent_map

    def update_stable_trail(self):
        self.stable_trail_map = gaussian_filter(self.stable_trail_map, sigma=0.0)

