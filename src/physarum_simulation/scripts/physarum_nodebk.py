#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from physarum.agent import Agent
from physarum.environment import Environment
from physarum.sensors import SensorSystem
from physarum.movement import MovementSystem
from physarum.deposition import DepositionSystem
from physarum.mass_update import MassSystem
from physarum.lifecycle import LifecycleSystem
from physarum.visualization import VisualizationSystem
from nav_msgs.msg import OccupancyGrid

import numpy as np

class PhysarumNode:
    def __init__(self):
        rospy.init_node('physarum_node', anonymous=True)
        self.robot_name = rospy.get_param("~robot_name", "robot_0")
        self.agent_count = rospy.get_param("~agent_count", 2000)
        self.other_robots = [f"robot_{i}" for i in range(0, 4) if f"robot_{i}" != self.robot_name]
        rospy.Subscriber('/signal_attenuation_map', OccupancyGrid, self.attenuation_callback)

        self.width = rospy.get_param("~width", 100)
        self.height = rospy.get_param("~height", 100)

        self.env = Environment(width=self.width, height=self.height)
        self.sensors = SensorSystem(sensing_distance=10, sensing_angle=np.pi/4)
        self.mover = MovementSystem(exploration_chance=0.5)
        self.depositor = DepositionSystem(base_amount=0.05)
        self.mass_updater = MassSystem(blur_sigma=0.0)
        self.lifecycle = LifecycleSystem(hunger_limit=10, reproduction_chance=0.1)
        self.attenuation_map = []

        self.other_poses = {}
        self.my_pose = {}

        self.agents = []
        
        
        self.vis = VisualizationSystem(self.env, self.agents)

        rospy.Subscriber(f"/{self.robot_name}/odom", Odometry, self.odom_callback)
        for other in self.other_robots:
            rospy.Subscriber(f"/{other}/odom", Odometry, self.make_other_callback(other))


    def attenuation_callback(self, msg):
        self.att_width = msg.info.width
        self.att_height = msg.info.height
        self.att_resolution = msg.info.resolution
        self.att_origin = msg.info.origin
        self.attenuation_map = np.array(msg.data).reshape((self.att_height, self.att_width))




    def is_visible_by_me(self, other_xy, sigma=20, intensity=2.0, threshold=0.005):
        """Verifica se o campo gaussiano do outro robô atinge o executor."""
        if self.robot_name not in self.my_pose:
            return False

        x0, y0 = self.my_pose[self.robot_name]
        x1, y1 = other_xy

        d = np.linalg.norm(np.array([x1, y1]) - np.array([x0, y0]))
        value = intensity * np.exp(-d**2 / (2 * sigma**2))
        return value >= threshold
        #self.timer = rospy.Timer(rospy.Duration(0.1), self.step)
    def world_to_map(self, x, y, width=100, height=100, x_min=-8, x_max=8, y_min=-8, y_max=8):
        mx = int((x - x_min) / (x_max - x_min) * width)
        my = height - int((y - y_min) / (y_max - y_min) * height)  # <- Inverte o eixo Y
        return mx, my
    
    def init_agents(self,qtd):
        spawn_x, spawn_y = self.my_pose[self.robot_name]
     
        spawn_x, spawn_y = self.world_to_map(spawn_x, spawn_y)
        for _ in range(qtd):
            offset_x = spawn_x + np.random.uniform(-3, 3)
            offset_y = spawn_y + np.random.uniform(-3, 3)
            heading = np.random.uniform(-np.pi, np.pi)
            self.agents.append(Agent(offset_y, offset_x, heading))

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if not self.robot_name in self.my_pose:
            self.my_pose[self.robot_name] = (x, y)
            #print(self.my_pose[self.robot_name])
            self.init_agents(self.agent_count)
        self.my_pose[self.robot_name] = (x, y)
            
        

    def make_other_callback(self, robot_name):
        def callback(msg):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.other_poses[robot_name] = (x, y)
        return callback



    def step(self, step):
        # Atualiza mapa de comida com posições atuais dos outros robôs
        #self.env.clear_food()
        self.env.food_map.fill(0)
        self.env.food_sources = []

     
  
        for other_pose in self.other_poses.values():
            if self.is_visible_by_me(other_pose, sigma=2, threshold=0.005):
                ox, oy = other_pose
                ox, oy = self.world_to_map(ox, oy)
                if self.attenuation_map is not None:
                    self.env.add_food(oy, ox, intensity=2.0, spread=20)
             
        if self.robot_name in self.my_pose:
            ox, oy = self.my_pose[self.robot_name]
            ox, oy = self.world_to_map(ox, oy)
            if self.attenuation_map is not None:
                self.env.add_food(oy, ox, intensity=2.0, spread=20)
                
            self.env.trail_map[oy, ox] += 1.0  # reforça constantemente
            self.env.mass_map[oy, ox] += 0.5 
        self.env.update_food(factor=0.3, min_spread=1.0, max_spread=20)
        self.mass_updater.update_mass(self.agents, self.env)

        for agent in self.agents:
            readings = self.sensors.sense(agent, self.env)
            self.mover.decide_and_move(agent, readings, self.env)
            self.depositor.deposit(agent, self.env)

        self.env.update_trail()
        self.env.update_stable_trail()
        self.vis.update(self.agents,step=step)
        
        if step % 100 == 0:
           
            # gerar novos agentes no robô 0 para reforçar a presença dele
          
            self.init_agents(100)
        if step % 1 == 0:
            sorted_agents = sorted(self.agents, key=lambda ag: ag.signal_strength)
            top_n = int(0.7* len(self.agents))
            for i, ag in enumerate(sorted_agents):
                ag.role = 'follower' if i < top_n else 'explorer'
       
    def run(self):
        rate = rospy.Rate(10)
        step = 0
        while not rospy.is_shutdown():
            
            self.step(step)
            step += 1
            rate.sleep()

if __name__ == '__main__':
    try:
        node = PhysarumNode()
        node.run()
    except rospy.ROSInterruptException:
        pass


import rospy
from nav_msgs.msg import Odometry
from physarum.agent import Agent
from physarum.environment import Environment
from physarum.sensors import SensorSystem
from physarum.movement import MovementSystem
from physarum.deposition import DepositionSystem
from physarum.mass_update import MassSystem
from physarum.lifecycle import LifecycleSystem
from physarum.visualization import VisualizationSystem

import numpy as np
from nav_msgs.msg import OccupancyGrid

class PhysarumNode:
    def __init__(self):
        rospy.init_node('physarum_node', anonymous=True)
        self.robot_name = rospy.get_param("~robot_name", "robot_0")
        self.agent_count = rospy.get_param("~agent_count", 2000)
        self.other_robots = [f"robot_{i}" for i in range(0, 4) if f"robot_{i}" != self.robot_name]
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        
        self.width = 100
        self.height = 100
        self.map_ready = False 
        self.env = Environment(width=self.width, height=self.height)
        self.sensors = SensorSystem(sensing_distance=10, sensing_angle=np.pi/4)
        self.mover = MovementSystem(exploration_chance=0.5)
        self.depositor = DepositionSystem(base_amount=0.05)
        self.mass_updater = MassSystem(blur_sigma=0.0)
        self.lifecycle = LifecycleSystem(hunger_limit=10, reproduction_chance=0.1)
        
        self.other_poses = {}
        self.my_pose = {}

        self.agents = []

        self.vis = VisualizationSystem(self.env, self.agents)

        rospy.Subscriber(f"/{self.robot_name}/odom", Odometry, self.odom_callback)
        for other in self.other_robots:
            rospy.Subscriber(f"/{other}/odom", Odometry, self.make_other_callback(other))


    def map_callback(self, msg):
            width = msg.info.width
            height = msg.info.height
            resolution = 0.16
            
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y

            data = np.array(msg.data).reshape((height, width))  # tipo int8
            data= np.flipud(data) 
            
            for y in range(height):
                for x in range(width):
                    value = data[y, x]
                    
                    if value >= 50:  # limiar típico para obstáculos
                        # converte para coordenadas do seu mapa (100x100)
                        map_x = ((x * resolution + origin_x + 8) / 16.0 * self.width)
                        map_y = ((y * resolution + origin_y + 8) / 16.0 * self.height)
                        if 0 <= map_x < self.width and 0 <= map_y < self.height:
                            self.env.add_obstacle(map_y, map_x, size=1)
            self.map_ready = True 




    def is_visible_by_me(self, other_xy, sigma=20, intensity=2.0, threshold=0.005):
        if self.robot_name not in self.my_pose:
            return False

        x0, y0 = self.my_pose[self.robot_name]
        x1, y1 = other_xy

        d = np.linalg.norm(np.array([x1, y1]) - np.array([x0, y0]))
        value = intensity * np.exp(-d**2 / (2 * sigma**2))
        return value >= threshold

    # def world_to_map(self, x, y, width=100, height=100, x_min=-8, x_max=8, y_min=-8, y_max=8):
    #     mx = int((x - x_min) / (x_max - x_min) * width)
    #     my = height - int((y - y_min) / (y_max - y_min) * height)
    #     return mx, my
    def world_to_map(self, x, y, resolution=0.16, origin_x=-8.0, origin_y=-8.0, width=100, height=100):
        mx = int((x - origin_x) / resolution)
        my = height -int((y - origin_y) / resolution)
        return mx, my
    # def world_to_map(self, x, y, width=100, height=100, x_min=-8.0, x_max=8.0, y_min=-8.0, y_max=8.0):
    #     mx = float((x - x_min) / (x_max - x_min) * width)
    #     my = height - float((y - y_min) / (y_max - y_min) * height)
   
    #     return mx, my

    def init_agents(self, qtd):
        spawn_x, spawn_y = self.my_pose[self.robot_name]
        spawn_x, spawn_y = self.world_to_map(spawn_x, spawn_y)
        for _ in range(qtd):
            offset_x = spawn_x + np.random.uniform(-3, 3)
            offset_y = spawn_y + np.random.uniform(-3, 3)
            heading = np.random.uniform(-np.pi, np.pi)
            self.agents.append(Agent(offset_y, offset_x, heading))

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if not self.robot_name in self.my_pose:
            self.my_pose[self.robot_name] = (x, y)
            self.init_agents(self.agent_count)
        self.my_pose[self.robot_name] = (x, y)

    def make_other_callback(self, robot_name):
        def callback(msg):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.other_poses[robot_name] = (x, y)
        return callback
  
    def step(self, step):
        self.env.food_maps_by_robot.clear()
        self.env.food_sources.clear()
        #self.add_containers()
        for robot_id, other_pose in self.other_poses.items():
            if self.is_visible_by_me(other_pose, sigma=2, threshold=0.005):
                ox, oy = other_pose
                ox, oy = self.world_to_map(ox, oy)
                self.env.food_sources.append((oy, ox))
                self.env.add_robot_signal(robot_id, oy, ox, intensity=2.0, spread=20)

        if self.robot_name in self.my_pose:
            ox, oy = self.my_pose[self.robot_name]
            ox, oy = self.world_to_map(ox, oy)
            
            self.env.food_sources.append((oy, ox))
            self.env.add_robot_signal(self.robot_name, oy, ox, intensity=2.0, spread=20)
            self.env.trail_map[int(oy), int(ox)] += 1.0
            self.env.mass_map[int(oy), int(ox)] += 0.5

        self.env.update_food()
        self.mass_updater.update_mass(self.agents, self.env)

        for agent in self.agents:
            readings = self.sensors.sense(agent, self.env)
            self.mover.decide_and_move(agent, readings, self.env)
            self.depositor.deposit(agent, self.env)

        self.env.update_trail()
        self.env.update_stable_trail()
        self.vis.update(self.agents, step=step)

        if step % 100 == 0:
            self.init_agents(100)

        if step % 1 == 0:
            sorted_agents = sorted(self.agents, key=lambda ag: ag.signal_strength)
            top_n = int(0.7 * len(self.agents))
            for i, ag in enumerate(sorted_agents):
                ag.role = 'follower' if i < top_n else 'explorer'

    def run(self):
        while not rospy.is_shutdown() and not self.map_ready:
            rospy.sleep(0.1) 
        rate = rospy.Rate(10)
        step = 0
        while not rospy.is_shutdown():
            self.step(step)
            step += 1
            rate.sleep()

if __name__ == '__main__':
    try:
        node = PhysarumNode()
        node.run()
    except rospy.ROSInterruptException:
        pass