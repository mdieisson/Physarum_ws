#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
pso_physarum_node.py — Versão PSO do seu PhysarumNode

Preserva os elementos do Physarum:
- Environment: food_map por robô, trail_map / stable_trail_map, mass_map, obstáculos
- DepositionSystem: reforço de trilhas
- MassSystem: massa por agentes
- VisualizationSystem: mesma visualização
- Publicação de conexões baseada em trilha (get_best_trail_connection)

Troca a lógica de movimento:
- Agentes viram partículas PSO (vx, vy, pbest, gbest)
- Fitness usa food + trail + mass - penalidade de obstáculo/repelente
"""

import rospy
import numpy as np

from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

from skimage.graph import route_through_array

from physarum.agent import Agent
from physarum.environment import Environment
from physarum.deposition import DepositionSystem
from physarum.mass_update import MassSystem
from physarum.lifecycle import LifecycleSystem
from physarum.visualization import VisualizationSystem


class PSOPhysarumNode:
    def __init__(self):
        rospy.init_node('pso_physarum_node_', anonymous=True)

        # -------- ROS params --------
        self.robot_name   = rospy.get_param("~robot_name", "robot_2")
        self.agent_count  = rospy.get_param("~agent_count", 2000)

        # PSO params (você pode ajustar em runtime via rosparam)
        self.w  = rospy.get_param("~pso_inertia_w", 0.72)          # inércia
        self.c1 = rospy.get_param("~pso_c1", 1.49)                 # cognitivo (pbest)
        self.c2 = rospy.get_param("~pso_c2", 1.49)                 # social (gbest)
        self.vmax = rospy.get_param("~pso_vmax", 1.2)              # velocidade máxima (grid step/frame)
        self.boundary_bounce = rospy.get_param("~pso_bounce", True)

        # Peso dos componentes no fitness (preserva “mesma ideia” do Physarum: comida/trilha/massa)
        self.k_food = rospy.get_param("~fit_food", 2.0)
        self.k_trail = rospy.get_param("~fit_trail", 0.6)
        self.k_mass = rospy.get_param("~fit_mass", 0.15)
        self.k_repellent = rospy.get_param("~fit_repellent", 2.5)  # penaliza regiões repulsivas/obstáculos

        # Pequeno “ruído” (mantém exploração)
        self.noise_sigma = rospy.get_param("~pso_noise_sigma", 0.03)

        # Mistura PSO com “seguir trilha” (opcional, mas ajuda a preservar estética do Physarum)
        self.trail_follow = rospy.get_param("~pso_trail_follow", 0.20)

        # -------- Rede e mapa --------
        self.other_robots = [f"robot_{i}" for i in range(0, 4) if f"robot_{i}" != self.robot_name]

        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber(f"/{self.robot_name}/odom", Odometry, self.odom_callback)
        for other in self.other_robots:
            rospy.Subscriber(f"/{other}/odom", Odometry, self.make_other_callback(other))

        self.connection_pub = rospy.Publisher(f'{self.robot_name}/physarum/connections', String, queue_size=10)

        self.width = 100
        self.height = 100
        self.map_ready = False

        self.env = Environment(width=self.width, height=self.height)
        self.depositor = DepositionSystem(base_amount=0.05)
        self.mass_updater = MassSystem(blur_sigma=0.0)
        self.lifecycle = LifecycleSystem(hunger_limit=10, reproduction_chance=0.1)

        self.other_poses = {}
        self.my_pose = {}

        self.agents = []
        self.vis = VisualizationSystem(self.env, self.agents, self.robot_name)

        # PSO global best
        self.gbest_pos = None          # (x, y) no grid
        self.gbest_score = -np.inf

        # Log (mantido)
        self.log_file = open(f"/tmp/{self.robot_name}_complexity_log.csv", "w")
        self.log_file.write("step,num_robots,potential_links,physarum_links,avg_signal_strength\n")

    # ----------------- Map & odom -----------------
    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = 0.16

        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        data = np.array(msg.data).reshape((height, width))
        data = np.flipud(data)

        for y in range(height):
            for x in range(width):
                value = data[y, x]
                if value >= 50:
                    map_x = ((x * resolution + origin_x + 8) / 16.0 * self.width)
                    map_y = ((y * resolution + origin_y + 8) / 16.0 * self.height)
                    if 0 <= map_x < self.width and 0 <= map_y < self.height:
                        self.env.add_obstacle(map_y, map_x, size=1)

        self.map_ready = True

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.robot_name not in self.my_pose:
            self.my_pose[self.robot_name] = (x, y)
            self.init_particles(self.agent_count)
        self.my_pose[self.robot_name] = (x, y)

    def make_other_callback(self, robot_name):
        def callback(msg):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.other_poses[robot_name] = (x, y)
        return callback

    def world_to_map(self, x, y, resolution=0.16, origin_x=-8.0, origin_y=-8.0, width=100, height=100):
        mx = int((x - origin_x) / resolution)
        my = height - int((y - origin_y) / resolution)
        return mx, my

    def is_visible_by_me(self, other_xy, sigma=20, intensity=2.0, threshold=0.005):
        if self.robot_name not in self.my_pose:
            return False
        x0, y0 = self.my_pose[self.robot_name]
        x1, y1 = other_xy
        d = np.linalg.norm(np.array([x1, y1]) - np.array([x0, y0]))
        value = intensity * np.exp(-d**2 / (2 * sigma**2))
        return value >= threshold

    # ----------------- Agents as PSO particles -----------------
    def init_particles(self, qtd):
        spawn_x, spawn_y = self.my_pose[self.robot_name]
        spawn_x, spawn_y = self.world_to_map(spawn_x, spawn_y)

        for _ in range(qtd):
            offset_x = spawn_x + np.random.uniform(-3, 3)
            offset_y = spawn_y + np.random.uniform(-3, 3)
            heading = np.random.uniform(-np.pi, np.pi)

            ag = Agent(offset_y, offset_x, heading)

            # AQUI: campos PSO adicionados (sem mexer no agent.py)
            ag.vx = np.random.uniform(-0.3, 0.3)
            ag.vy = np.random.uniform(-0.3, 0.3)
            ag.pbest_x = ag.x
            ag.pbest_y = ag.y
            ag.pbest_score = -np.inf

            self.agents.append(ag)

    # ----------------- Trail / connections (mantidos) -----------------
    def get_robot_centers(self):
        centers = {}
        for robot_id, food_map in self.env.food_maps_by_robot.items():
            y, x = np.unravel_index(np.argmax(food_map), food_map.shape)
            centers[robot_id] = (x, y)
        return centers

    def count_active_links(self, centers, threshold=0.2):
        active_count = 0
        total_signal = 0
        count = 0

        own_id = self.robot_name
        if own_id not in centers:
            return 0, 0

        for other_id, other_pos in centers.items():
            if other_id == own_id:
                continue

            try:
                cost_map = np.max(self.env.stable_trail_map) - self.env.stable_trail_map + 1e-6
                path, _ = route_through_array(cost_map, centers[own_id], other_pos, fully_connected=True)
                mean_trail = np.mean([self.env.stable_trail_map[y, x] for y, x in path])

                total_signal += mean_trail
                count += 1
                if mean_trail > threshold:
                    active_count += 1
            except Exception:
                continue

        avg_signal = (total_signal / count) if count > 0 else 0
        return active_count, avg_signal

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
                cost_map = np.max(self.env.trail_map) - self.env.trail_map + 1e-6
                path, _ = route_through_array(cost_map, centers[own_id], other_pos, fully_connected=True)

                # quanto “melhor” o tubo, menor o cost_map; então invertemos pra score
                mean_cost = np.mean([cost_map[y, x] for y, x in path])
                score = -mean_cost
                if score > best_score:
                    best_score = score
                    best_conn = (own_id, other_id)
            except Exception:
                continue

        return best_conn

    def publish_connections(self, conn_str):
        msg = String()
        msg.data = conn_str
        self.connection_pub.publish(msg)

    # ----------------- PSO core -----------------
    def _in_bounds(self, x, y):
        return 0 <= int(x) < self.height and 0 <= int(y) < self.width

    def _is_obstacle(self, x, y):
        xi, yi = int(x), int(y)
        if not self._in_bounds(xi, yi):
            return True
        return bool(self.env.obstacle_map[xi, yi])

    def fitness(self, x, y):
        if not self._in_bounds(x, y):
            return -1e9

        mass, trail, food = self.env.sense(x, y)

        # repellent_map (se existir) + obstáculo duro
        rep = 0.0
        xi, yi = int(x), int(y)
        if hasattr(self.env, "repellent_map"):
            rep = float(self.env.repellent_map[xi, yi])

        obstacle_penalty = 50.0 if self._is_obstacle(x, y) else 0.0

        # Fitness alto = bom (atrai)
        return (
            self.k_food * float(food) +
            self.k_trail * float(trail) +
            self.k_mass * float(mass) -
            self.k_repellent * abs(float(rep)) -
            obstacle_penalty
        )

    def local_trail_gradient(self, x, y):
        """Gradiente simples (diferença central) para ajudar a “colar” na trilha."""
        xi, yi = int(x), int(y)
        if not self._in_bounds(xi, yi):
            return 0.0, 0.0

        tm = self.env.trail_map
        # clamp índices
        xm1 = max(xi - 1, 0); xp1 = min(xi + 1, self.height - 1)
        ym1 = max(yi - 1, 0); yp1 = min(yi + 1, self.width - 1)

        gx = float(tm[xp1, yi] - tm[xm1, yi])
        gy = float(tm[xi, yp1] - tm[xi, ym1])
        return gx, gy

    def pso_step(self):
        # 1) Avalia fitness, atualiza pbest e gbest
        for ag in self.agents:
            s = self.fitness(ag.x, ag.y)
            ag.signal_strength = s  # reaproveita esse campo no seu pipeline

            if s > ag.pbest_score:
                ag.pbest_score = s
                ag.pbest_x = ag.x
                ag.pbest_y = ag.y

            if s > self.gbest_score:
                self.gbest_score = s
                self.gbest_pos = (ag.x, ag.y)

        if self.gbest_pos is None:
            return

        gbest_x, gbest_y = self.gbest_pos

        # 2) Atualiza velocidade e posição (PSO)
        for ag in self.agents:
            r1 = np.random.rand()
            r2 = np.random.rand()

            # papel explorer => mais ruído, follower => mais “social”
            role_boost = 1.25 if ag.role == "explorer" else 1.0
            c2_eff = self.c2 * (1.2 if ag.role != "explorer" else 0.9)

            # componentes PSO
            cognitive_x = self.c1 * r1 * (ag.pbest_x - ag.x)
            cognitive_y = self.c1 * r1 * (ag.pbest_y - ag.y)
            social_x    = c2_eff * r2 * (gbest_x - ag.x)
            social_y    = c2_eff * r2 * (gbest_y - ag.y)

            # “seguir trilha” (mantém estética Physarum)
            tgx, tgy = self.local_trail_gradient(ag.x, ag.y)
            trail_x = self.trail_follow * tgx
            trail_y = self.trail_follow * tgy

            # ruído
            nx = np.random.normal(0, self.noise_sigma) * role_boost
            ny = np.random.normal(0, self.noise_sigma) * role_boost

            ag.vx = self.w * ag.vx + cognitive_x + social_x + trail_x + nx
            ag.vy = self.w * ag.vy + cognitive_y + social_y + trail_y + ny

            # limita velocidade
            ag.vx = float(np.clip(ag.vx, -self.vmax, self.vmax))
            ag.vy = float(np.clip(ag.vy, -self.vmax, self.vmax))

            # aplica movimento
            new_x = ag.x + ag.vx
            new_y = ag.y + ag.vy

            # colisão com limites/obstáculo
            if not self._in_bounds(new_x, new_y) or self._is_obstacle(new_x, new_y):
                if self.boundary_bounce:
                    # rebate e “solta” um pouco aleatório
                    ag.vx *= -0.6
                    ag.vy *= -0.6
                    new_x = np.clip(ag.x + ag.vx + np.random.uniform(-0.5, 0.5), 0, self.height - 1)
                    new_y = np.clip(ag.y + ag.vy + np.random.uniform(-0.5, 0.5), 0, self.width - 1)
                    # se ainda for obstáculo, teleporta curtinho
                    if self._is_obstacle(new_x, new_y):
                        new_x = np.clip(ag.x + np.random.uniform(-2, 2), 0, self.height - 1)
                        new_y = np.clip(ag.y + np.random.uniform(-2, 2), 0, self.width - 1)
                else:
                    # alternativa: respawn leve
                    new_x = np.clip(ag.x + np.random.uniform(-2, 2), 0, self.height - 1)
                    new_y = np.clip(ag.y + np.random.uniform(-2, 2), 0, self.width - 1)

            # atualiza prev (mantém DepositionSystem compatível com o seu histórico)
            ag.prev_x = ag.x
            ag.prev_y = ag.y
            ag.x = float(new_x)
            ag.y = float(new_y)

            # atualiza heading (só pra visual/consistência)
            ag.heading = float(np.arctan2(ag.vy, ag.vx))

    # ----------------- Main loop step -----------------
    def step(self, step):
        # limpa sinais/“comida” por robô (igual você faz)
        self.env.food_maps_by_robot.clear()
        self.env.food_sources.clear()

        # 1) atualiza food_map com robôs visíveis (mesma ideia do seu node)
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

            # mantém “âncora” de trilha/massa no próprio robô (igual seu código)
            self.env.trail_map[int(oy), int(ox)] += 1.0
            self.env.mass_map[int(oy), int(ox)] += 0.5

        self.env.update_food()

        # 2) atualiza massa (mesmo módulo do Physarum)
        self.mass_updater.update_mass(self.agents, self.env)

        # 3) PSO: move partículas
        self.pso_step()

        # 4) deposição (preserva trilhas do Physarum)
        for ag in self.agents:
            self.depositor.deposit(ag, self.env)

        # 5) atualiza trilhas (decay + stable)
        self.env.update_trail()
        self.env.update_stable_trail()

        # 6) mantém sua lógica de “re-semeadura” e papéis
        if step % 100 == 0:
            self.init_particles(100)

        if step % 1 == 0:
            sorted_agents = sorted(self.agents, key=lambda a: a.signal_strength)
            top_n = int(0.7 * len(self.agents))
            for i, ag in enumerate(sorted_agents):
                ag.role = 'follower' if i < top_n else 'explorer'

        # 7) conexões por trilha (igual você faz)
        centers = self.get_robot_centers()

        num_neighbors = len(centers) - 1
        potential_links = max(0, num_neighbors)

        active_physarum, avg_sig = self.count_active_links(centers, threshold=0.2)
        if step % 10 == 0:
            self.log_file.write(f"{step},{num_neighbors+1},{potential_links},{active_physarum},{avg_sig:.4f}\n")
            self.log_file.flush()

        conn = self.get_best_trail_connection(centers)
        if conn:
            self.publish_connections(conn[1])
        else:
            self.publish_connections("")

        # 8) visualização
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
        node = PSOPhysarumNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
