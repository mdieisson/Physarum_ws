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

import numpy as np
from nav_msgs.msg import OccupancyGrid
import networkx as nx
from std_msgs.msg import String
from skimage.graph import route_through_array
class PhysarumNode:
    def __init__(self):
        rospy.init_node('physarum_node_', anonymous=True)

        self.robot_name = rospy.get_param("~robot_name", "robot_2")
        self.agent_count = rospy.get_param("~agent_count", 2000)
        self.other_robots = [f"robot_{i}" for i in range(0, 4) if f"robot_{i}" != self.robot_name]
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.connection_pub = rospy.Publisher(f'{self.robot_name}/physarum/connections', String, queue_size=10)                                                                                                                               
        
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

        self.vis = VisualizationSystem(self.env, self.agents, self.robot_name)

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

    def get_robot_centers(self):
        centers = {}
        for robot_id, food_map in self.env.food_maps_by_robot.items():
            y, x = np.unravel_index(np.argmax(food_map), food_map.shape)
            centers[robot_id] = (x, y)
        return centers

    def build_trail_graph(self, threshold=0.2):
        trail_map = self.env.trail_map
        G = nx.Graph()
        h, w = trail_map.shape
        for y in range(h):
            for x in range(w):
                if trail_map[y, x] > threshold:
                    for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                        nx_, ny_ = x + dx, y + dy
                        if 0 <= nx_ < w and 0 <= ny_ < h and trail_map[ny_, nx_] > threshold:
                            G.add_edge((x, y), (nx_, ny_))
        return G
    

    def get_best_trail_connection(self, centers):
        own_id = self.robot_name
        if own_id not in centers:
            return None

        best_conn = None
        best_score = -np.inf

        for other_id, other_pos in centers.items():
            if other_id == own_id:
                continue

            try:
                # A estratégia do Physarum é seguir trilhas reforçadas — por isso invertemos os valores:
                cost_map = np.max(self.env.stable_trail_map) - self.env.stable_trail_map + 1e-6
                path, cost = route_through_array(cost_map, centers[own_id], other_pos, fully_connected=True)
                mean_trail = np.mean([self.env.stable_trail_map[y, x] for y, x in path])
                if mean_trail > best_score:
                    best_score = mean_trail
                    best_conn = (own_id, other_id)
            except Exception:
                continue  # Se não existe caminho, ignora

        return best_conn



    # def get_robot_connections(self, G, centers):
    #     connections = []
    #     ids = list(centers.keys())
    #     for i in range(len(ids)):
    #         for j in range(i + 1, len(ids)):
    #             a, b = ids[i], ids[j]
    #             if centers[a] in G and centers[b] in G:
    #                 if nx.has_path(G, centers[a], centers[b]):
    #                     connections.append((a, b))
    #     return connections

    def publish_connections(self, connections):
        msg = String()
        #if connections !="":
        msg.data = connections
        self.connection_pub.publish(msg)
        #else:
           # msg.data = ""
           # self.connection_pub.publish(msg)

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
        

        if step % 100 == 0:
            self.init_agents(100)

        if step % 1 == 0:
            sorted_agents = sorted(self.agents, key=lambda ag: ag.signal_strength)
            top_n = int(0.7 * len(self.agents))
            for i, ag in enumerate(sorted_agents):
                ag.role = 'follower' if i < top_n else 'explorer'

        centers = self.get_robot_centers()
        G = self.build_trail_graph()
        connections = self.get_best_trail_connection(centers)
        if connections:
            self.publish_connections(connections[1])
        else:
           self.publish_connections("") 

        #self.vis.draw_connections(centers, connections)
        self.vis.fig.canvas.draw()

        self.vis.update(self.agents, step=step)
    

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
