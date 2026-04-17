#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
aco_node.py — ACO dinâmico (online) EMERGENTE para formação de conexões em rede entre robôs,
no MESMO cenário do physarum_node.

Modo emergente:
- Nenhuma formiga tem alvo fixo.
- As formigas seguem estocasticamente:
    (i) feromônio (trail_map / stable_trail_map) e
    (ii) heurística global (food_map) = soma dos "sinais" (Wi-Fi/Bluetooth) dos robôs visíveis.
- Depósito contínuo fraco a cada passo (forma "tubos").
- Depósito bônus quando a formiga passa perto de QUALQUER robô visível (reforça caminhos entre fontes).
- Máscara do próprio robô no food_map para evitar padrão de "raios" saindo do centro.
- Evaporação contínua (online).

Publica em: /<robot_name>/aco/connections  (String)
com o ID do robô ao qual existe a "melhor" conexão (maior feromônio médio no caminho geodésico).
"""

import rospy
import numpy as np
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String

from physarum.environment import Environment  # reuso do mesmo ambiente
from physarum.visualization import VisualizationSystem  # opcional (visualização)

from skimage.graph import route_through_array


class Ant:
    """Formiga discreta no grid (x,y em coordenadas do mapa 100x100)."""

    __slots__ = ("x", "y", "heading", "path", "steps", "max_steps")

    def __init__(self, x, y, heading, max_steps=200):
        self.x = float(x)
        self.y = float(y)
        self.heading = float(heading)
        self.path = [(int(round(self.y)), int(round(self.x)))]  # (row=y, col=x)
        self.steps = 0
        self.max_steps = max_steps

    def pos_int(self, h, w):
        cx = int(np.clip(int(round(self.x)), 0, w - 1))
        cy = int(np.clip(int(round(self.y)), 0, h - 1))
        return cx, cy


class ACONode:
    def __init__(self):
        rospy.init_node("aco_node", anonymous=True)

        # ---- params ----
        self.robot_name = rospy.get_param("~robot_name", "robot_2")

        # população e dinâmica
        self.ant_count = rospy.get_param("~ant_count", 30)
        self.spawn_per_step = rospy.get_param("~spawn_per_step", 10)
        self.max_ants = rospy.get_param("~max_ants", 200)
        self.max_steps_per_ant = rospy.get_param("~max_steps_per_ant", 400)

        # ACO (emergente)
        self.alpha = rospy.get_param("~alpha", 1.0)              # peso do feromônio
        self.beta = rospy.get_param("~beta", 2.0)                # peso do sinal (food_map)
        self.evaporation = rospy.get_param("~evaporation", 0.98) # ver nota abaixo (depende do Environment)
        self.q_deposit = rospy.get_param("~q_deposit", 1.0)      # intensidade base de depósito (bônus)
        self.deposit_step = rospy.get_param("~deposit_step", 0.01)   # depósito contínuo por passo
        self.deposit_bonus = rospy.get_param("~deposit_bonus", 2.0)  # bônus ao passar perto de fonte

        self.step_size = rospy.get_param("~step_size", 1.0)
        self.sensing_distance = rospy.get_param("~sensing_distance", 2)

        # evita “raios” presos no próprio robô
        self.self_mask_radius = rospy.get_param("~self_mask_radius", 6)

        # conexão
        self.link_threshold = rospy.get_param("~link_threshold", 0.2)

        # mapa interno
        self.width = rospy.get_param("~grid_width", 100)
        self.height = rospy.get_param("~grid_height", 100)

        self.map_ready = False

        # ---- environment ----
        # IMPORTANTE: se no seu Environment o decay for "taxa", ajuste evaporation para algo tipo 0.02.
        # Se for "multiplicador", 0.98 costuma funcionar bem.
        self.env = Environment(width=self.width, height=self.height, decay_factor=self.evaporation)

        # Visualização (mantém o mesmo estilo do Physarum)
        self.ants = []
        self.vis = VisualizationSystem(self.env, [], self.robot_name, view_mode="trail")

        # ---- ROS I/O ----
        self.other_robots = [f"robot_{i}" for i in range(0, 4) if f"robot_{i}" != self.robot_name]

        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.Subscriber(f"/{self.robot_name}/odom", Odometry, self.odom_callback)

        for other in self.other_robots:
            rospy.Subscriber(f"/{other}/odom", Odometry, self.make_other_callback(other))

        # tópico ABSOLUTO
        self.connection_pub = rospy.Publisher(f"/{self.robot_name}/aco/connections", String, queue_size=10)

        # logging (mesmo formato do physarum_node)
        self.log_file = open(f"/tmp/{self.robot_name}_aco_complexity_log.csv", "w")
        self.log_file.write("step,num_robots,potential_links,aco_links,avg_pheromone_strength\n")

        self.other_poses = {}
        self.my_pose = {}

        # convenção do mapa do cenário (mesma do physarum_node)
        self.resolution = 0.16
        self.origin_x = -8.0
        self.origin_y = -8.0

    # ---------------------------- util ----------------------------

    def world_to_map(self, x, y, resolution=0.16, origin_x=-8.0, origin_y=-8.0, width=100, height=100):
        mx = int((x - origin_x) / resolution)
        my = height - int((y - origin_y) / resolution)
        return mx, my  # (x,y) no grid

    def is_visible_by_me(self, other_xy, sigma=2.0, intensity=2.0, threshold=0.005):
        """Mesmo modelo de visibilidade do Physarum (decai com a distância)."""
        if self.robot_name not in self.my_pose:
            return False
        x0, y0 = self.my_pose[self.robot_name]
        x1, y1 = other_xy
        d = np.linalg.norm(np.array([x1, y1]) - np.array([x0, y0]))
        value = intensity * np.exp(-d**2 / (2 * sigma**2))
        return value >= threshold

    def _neighbors_heading(self, ant):
        """Retorna 3 headings (forward/left/right) e suas posições candidatas."""
        base = ant.heading
        deltas = [0.0, np.pi / 4, -np.pi / 4]
        cand = []
        for d in deltas:
            ang = base + d
            nx = ant.x + self.step_size * np.cos(ang)
            ny = ant.y + self.step_size * np.sin(ang)
            cand.append((ang, nx, ny))
        return cand

    def _heuristic(self, x, y):
        """Heurística η baseada no food_map global (sinais dos robôs)."""
        xi = int(np.clip(int(round(x)), 0, self.width - 1))
        yi = int(np.clip(int(round(y)), 0, self.height - 1))
        return float(max(self.env.food_map[yi, xi], 0.0)) + 1e-6

    def _pheromone(self, x, y):
        """Retorna feromônio no ponto (clamp)."""
        xi = int(np.clip(int(round(x)), 0, self.width - 1))
        yi = int(np.clip(int(round(y)), 0, self.height - 1))
        return float(max(self.env.trail_map[yi, xi], 0.0))

    def _is_free(self, x, y):
        xi = int(np.clip(int(round(x)), 0, self.width - 1))
        yi = int(np.clip(int(round(y)), 0, self.height - 1))
        return 0 <= xi < self.width and 0 <= yi < self.height and (not self.env.obstacle_map[yi, xi])

    def _mask_self_signal(self, centers):
        """Abafa o sinal perto do próprio robô para evitar 'raios' saindo do centro."""
        if self.robot_name not in centers:
            return
        mx, my = centers[self.robot_name]  # (x,y)
        r = int(self.self_mask_radius)

        x0 = int(np.clip(mx, 0, self.width - 1))
        y0 = int(np.clip(my, 0, self.height - 1))
        y_min, y_max = max(0, y0 - r), min(self.height, y0 + r + 1)
        x_min, x_max = max(0, x0 - r), min(self.width,  x0 + r + 1)

        yy, xx = np.ogrid[y_min:y_max, x_min:x_max]
        mask = (xx - x0) ** 2 + (yy - y0) ** 2 <= r * r
        self.env.food_map[y_min:y_max, x_min:x_max][mask] *= 0.05

    def _near_any_source(self, ant, centers):
        """True se a formiga estiver perto de qualquer robô visível (fonte)."""
        ax, ay = ant.x, ant.y
        for _, (mx, my) in centers.items():
            if np.hypot(mx - ax, my - ay) <= self.sensing_distance:
                return True
        return False

    # ---------------------------- ROS callbacks ----------------------------

    def map_callback(self, msg):
        """Carrega obstáculos do /map para obstacle_map do Environment."""
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution if msg.info.resolution > 0 else self.resolution

        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int16).reshape((height, width))
        data = np.flipud(data)

        # (opcional) se houver múltiplos callbacks, isso evita acumular obstáculo:
        # self.env.obstacle_map[:] = False

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
        self.my_pose[self.robot_name] = (x, y)

        # inicializa população de formigas uma vez
        if len(self.ants) == 0:
            self._spawn_ants(self.ant_count)

    def make_other_callback(self, robot_name):
        def callback(msg):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.other_poses[robot_name] = (x, y)
        return callback

    # ---------------------------- ACO core (emergente) ----------------------------

    def _visible_robot_centers(self):
        """
        Retorna dict robot_id -> (mx,my) apenas dos robôs visíveis (inclui o próprio).
        """
        centers = {}

        for rid, pose in self.other_poses.items():
            if self.is_visible_by_me(pose):
                mx, my = self.world_to_map(pose[0], pose[1], resolution=self.resolution,
                                           origin_x=self.origin_x, origin_y=self.origin_y,
                                           width=self.width, height=self.height)
                centers[rid] = (mx, my)

        if self.robot_name in self.my_pose:
            pose = self.my_pose[self.robot_name]
            mx, my = self.world_to_map(pose[0], pose[1], resolution=self.resolution,
                                       origin_x=self.origin_x, origin_y=self.origin_y,
                                       width=self.width, height=self.height)
            centers[self.robot_name] = (mx, my)

        return centers

    def _spawn_ants(self, n):
        """Cria n formigas ao redor do robô local."""
        if self.robot_name not in self.my_pose:
            return
        rx, ry = self.my_pose[self.robot_name]
        mx, my = self.world_to_map(rx, ry, resolution=self.resolution,
                                  origin_x=self.origin_x, origin_y=self.origin_y,
                                  width=self.width, height=self.height)

        for _ in range(int(n)):
            ox = mx + np.random.uniform(-3, 3)
            oy = my + np.random.uniform(-3, 3)
            heading = np.random.uniform(-np.pi, np.pi)
            self.ants.append(Ant(ox, oy, heading, max_steps=self.max_steps_per_ant))

    def _step_ant(self, ant):
        """Um passo de decisão e movimento probabilístico ACO emergente."""
        cand = self._neighbors_heading(ant)
        weights = []

        for ang, nx, ny in cand:
            if not self._is_free(nx, ny):
                weights.append(0.0)
                continue

            tau = self._pheromone(nx, ny) + 1e-6
            eta = self._heuristic(nx, ny) + 1e-6
            w = (tau ** self.alpha) * (eta ** self.beta)
            weights.append(w)

        total = float(sum(weights))
        if total <= 0.0:
            # travado -> tenta mover aleatório (não só girar)
            ang = ant.heading + np.random.uniform(-np.pi, np.pi)
            nx = ant.x + self.step_size * np.cos(ang)
            ny = ant.y + self.step_size * np.sin(ang)
            if self._is_free(nx, ny):
                ant.heading = ang
                ant.x, ant.y = nx, ny
            ant.steps += 1
            return

        probs = [w / total for w in weights]
        idx = np.random.choice(len(cand), p=probs)
        ang, nx, ny = cand[idx]

        ant.heading = ang
        ant.x, ant.y = nx, ny
        ant.steps += 1

        yi = int(np.clip(int(round(ant.y)), 0, self.height - 1))
        xi = int(np.clip(int(round(ant.x)), 0, self.width - 1))
        ant.path.append((yi, xi))

        # depósito contínuo fraco (forma "tubos")
        if not self.env.obstacle_map[yi, xi]:
            self.env.trail_map[yi, xi] += self.deposit_step
            self.env.stable_trail_map[yi, xi] += 0.3 * self.deposit_step

    def _deposit_bonus_path(self, ant):
        """Bônus de feromônio ao longo do caminho (quando passa perto de uma fonte)."""
        if not ant.path:
            return
        L = max(len(ant.path), 1)
        delta = (self.q_deposit * self.deposit_bonus) / float(L)
        for (y, x) in ant.path:
            if 0 <= y < self.height and 0 <= x < self.width and not self.env.obstacle_map[y, x]:
                self.env.trail_map[y, x] += delta
                self.env.stable_trail_map[y, x] += 0.5 * delta

    def _evaporate(self):
        """Evaporação online."""
        self.env.update_trail()
        self.env.update_stable_trail()

    # ---------------------------- conexão (extração da topologia) ----------------------------

    def _count_active_links(self, centers, threshold=0.2):
        """Conta quantos links do robô local passam por feromônio "forte" (via caminho geodésico)."""
        active = 0
        total_signal = 0.0
        count = 0

        own_id = self.robot_name
        if own_id not in centers:
            return 0, 0.0

        max_pher = float(np.max(self.env.stable_trail_map))
        if max_pher <= 1e-9:
            return 0, 0.0

        for other_id, other_pos in centers.items():
            if other_id == own_id:
                continue
            try:
                # ATENÇÃO: route_through_array espera (row, col) = (y, x)
                start = (centers[own_id][1], centers[own_id][0])
                goal = (other_pos[1], other_pos[0])

                cost_map = max_pher - self.env.stable_trail_map + 1e-6
                path, _ = route_through_array(cost_map, start, goal, fully_connected=True)
                mean_pher = float(np.mean([self.env.stable_trail_map[y, x] for y, x in path]))
                total_signal += mean_pher
                count += 1
                if mean_pher > threshold:
                    active += 1
            except Exception:
                continue

        avg_signal = (total_signal / count) if count > 0 else 0.0
        return active, avg_signal

    def _best_connection(self, centers):
        """Retorna (own_id, best_other_id) baseado no maior feromônio médio em stable_trail_map."""
        own_id = self.robot_name
        if own_id not in centers:
            return None

        max_pher = float(np.max(self.env.stable_trail_map))
        if max_pher <= 1e-9:
            return None

        best_other = None
        best_score = -np.inf

        for other_id, other_pos in centers.items():
            if other_id == own_id:
                continue
            try:
                start = (centers[own_id][1], centers[own_id][0])
                goal = (other_pos[1], other_pos[0])

                cost_map = max_pher - self.env.stable_trail_map + 1e-6
                path, _ = route_through_array(cost_map, start, goal, fully_connected=True)
                mean_pher = float(np.mean([self.env.stable_trail_map[y, x] for y, x in path]))
                if mean_pher > best_score:
                    best_score = mean_pher
                    best_other = other_id
            except Exception:
                continue

        if best_other is None:
            return None
        return (own_id, best_other)

    def _publish_connection(self, other_id):
        msg = String()
        msg.data = other_id if other_id is not None else ""
        self.connection_pub.publish(msg)

    # ---------------------------- main loop ----------------------------

    def step(self, step_idx):
        # 1) centros visíveis (inclui próprio)
        centers = self._visible_robot_centers()

        # 1.1) Atualiza o food_map (campo de sinal) no mesmo estilo do physarum_node
        # limpa estruturas compatíveis (se existirem)
        if hasattr(self.env, "food_maps_by_robot"):
            self.env.food_maps_by_robot.clear()
        if hasattr(self.env, "food_sources"):
            self.env.food_sources.clear()

        # adiciona sinais dos outros robôs visíveis
        for rid, pose in self.other_poses.items():
            if self.is_visible_by_me(pose):
                mx, my = self.world_to_map(pose[0], pose[1], resolution=self.resolution,
                                           origin_x=self.origin_x, origin_y=self.origin_y,
                                           width=self.width, height=self.height)
                if hasattr(self.env, "food_sources"):
                    self.env.food_sources.append((my, mx))
                self.env.add_robot_signal(rid, my, mx, intensity=2.0, spread=20.0)

        # adiciona sinal do próprio robô (pode manter por consistência com Physarum)
        if self.robot_name in self.my_pose:
            pose = self.my_pose[self.robot_name]
            mx, my = self.world_to_map(pose[0], pose[1], resolution=self.resolution,
                                       origin_x=self.origin_x, origin_y=self.origin_y,
                                       width=self.width, height=self.height)
            if hasattr(self.env, "food_sources"):
                self.env.food_sources.append((my, mx))
            self.env.add_robot_signal(self.robot_name, my, mx, intensity=2.0, spread=20.0)

        # combina mapas individuais -> food_map
        self.env.update_food()

        # máscara do sinal do próprio robô (reduz "estrela/raios")
        self._mask_self_signal(centers)

        # 2) spawn incremental (dinâmico, online)
        if step_idx % 1 == 0 and len(self.ants) < self.max_ants:
            self._spawn_ants(self.spawn_per_step)

        # 3) simula formigas (um passo por iteração) — emergente
        survivors = []
        for ant in self.ants:
            self._step_ant(ant)

            # ao passar perto de qualquer fonte, reforça o caminho e reinicia
            if self._near_any_source(ant, centers):
                self._deposit_bonus_path(ant)
                if self.robot_name in centers:
                    mx, my = centers[self.robot_name]
                    ant.x = mx + np.random.uniform(-3, 3)
                    ant.y = my + np.random.uniform(-3, 3)
                    ant.heading = np.random.uniform(-np.pi, np.pi)
                    ant.path = [(int(round(ant.y)), int(round(ant.x)))]
                    ant.steps = 0
                    survivors.append(ant)
                continue

            if ant.steps < ant.max_steps:
                survivors.append(ant)

        self.ants = survivors

        # 4) evaporação
        self._evaporate()

        # 5) logs de "complexidade" (comparável ao seu csv)
        num_neighbors = max(0, len(centers) - 1)
        potential_links = num_neighbors

        active_aco, avg_sig = self._count_active_links(centers, threshold=self.link_threshold)
        if step_idx % 10 == 0:
            self.log_file.write(f"{step_idx},{num_neighbors+1},{potential_links},{active_aco},{avg_sig:.4f}\n")
            self.log_file.flush()

        # 6) publica melhor conexão
        conn = self._best_connection(centers)
        if conn is not None:
            self._publish_connection(conn[1])
        else:
            self._publish_connection(None)

        # 7) visualização
        try:
            self.vis.update([], step=step_idx)
        except Exception:
            pass

    def run(self):
        while not rospy.is_shutdown() and not self.map_ready:
            rospy.sleep(0.1)

        rate = rospy.Rate(10)
        step_idx = 0
        while not rospy.is_shutdown():
            self.step(step_idx)
            step_idx += 1
            rate.sleep()


if __name__ == "__main__":
    try:
        node = ACONode()
        node.run()
    except rospy.ROSInterruptException:
        pass
