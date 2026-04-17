#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
aco_field_node.py — ACO em CAMPO (grid) para comparação com Physarum.

Objetivo: fazer as "formigas" se comportarem como agentes locais que seguem gradientes de sinal (RSSI),
sem "saltar" diretamente para um robô vizinho.

✅ A lógica do ACO usa SOMENTE campos (mapas) de sinal:
   - S(x,y)  (env.food_map)  ou
   - S_j(x,y) (env.food_maps_by_robot[j]) para extração de conexões
✅ As formigas caminham célula a célula (8-vizinhança) no grid.
✅ "Chegada" = pico local de sinal (threshold + máximo local).

⚠️ Para obter os mapas de sinal dos robôs (gaussianas), há 2 modos:
   1) (Padrão) use_pose_sensor=True:
        Este nó assina /odom dos robôs e atualiza env.food_maps_by_robot como SENSOR do simulador.
        O ACO NÃO usa poses; usa apenas os MAPAS resultantes.
   2) use_pose_sensor=False:
        Você deve alimentar env.food_map / env.food_maps_by_robot externamente (ex.: por outro nó).

Visualização:
- Reusa physarum.environment.Environment + physarum.visualization.VisualizationSystem
- Usa env.trail_map como tau_map (feromônio) para visualização.

Publica:
- /<robot_name>/aco_field/connections (std_msgs/String): melhor robô (via W_j = sum(tau * S_j))
"""

import math
import rospy
import numpy as np
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String

from physarum.environment import Environment
from physarum.visualization import VisualizationSystem


class VizAnt:
    """Ant wrapper só para compatibilizar com VisualizationSystem.
    No seu visualization.py, agent.x é tratado como LINHA (y) e agent.y como COLUNA (x).
    Aqui mantemos interno (x,col) e (y,lin), então para plot invertimos.
    """
    __slots__ = ("x","y")
    def __init__(self, row, col):
        self.x = int(row)  # linha
        self.y = int(col)  # coluna


class FieldAnt:
    """Formiga como agente no grid."""
    __slots__ = ("x", "y", "path", "steps", "visited")
    def __init__(self, x, y):
        self.x = int(x)
        self.y = int(y)
        self.path = [(int(x), int(y))]
        self.steps = 0
        self.visited = set(self.path)


class AcoFieldNode:
    def __init__(self):
        rospy.init_node("aco_field_node", anonymous=True)

        # --- identidade ---
        self.robot_name = rospy.get_param("~robot_name", "robot_2")
        self.num_robots_total = rospy.get_param("~num_robots_total", 4)

        # --- grid ---
        self.width = rospy.get_param("~width", 100)
        self.height = rospy.get_param("~height", 100)

        # --- ACO (campo) ---
        self.alpha = float(rospy.get_param("~alpha", 1.0))          # peso do feromônio
        self.beta  = float(rospy.get_param("~beta",  3.0))          # peso do sinal
        self.rho   = float(rospy.get_param("~evaporation", 0.95))   # evaporação do feromônio
        self.q     = float(rospy.get_param("~deposit_q", 0.05))      # depósito
        self.diffuse = float(rospy.get_param("~diffuse", 1.0))     # difusão suave do feromônio (0..1)
        self.max_steps = int(rospy.get_param("~ant_max_steps", 120))
        self.ants_per_cycle = int(rospy.get_param("~ants_per_cycle", 120))

        # depósito exploratório (para trilhas aparecerem mesmo sem atingir pico)
        self.deposit_on_fail = bool(rospy.get_param("~deposit_on_fail", True))
        self.fail_scale = float(rospy.get_param("~fail_scale", 0.01))

        # --- critério de "pico" ---
        self.peak_threshold = float(rospy.get_param("~peak_threshold", 0.15))  # sinal mínimo
        self.peak_margin = float(rospy.get_param("~peak_margin", 0.02))        # maior que vizinhos por margem

        # --- sensor de sinal (apenas para construir food_map) ---
        self.use_pose_sensor = bool(rospy.get_param("~use_pose_sensor", True))
        self.comm_intensity = float(rospy.get_param("~comm_intensity", 2.0))
        self.signal_spread = int(rospy.get_param("~signal_spread", 10))
        self.include_self_signal = bool(rospy.get_param("~include_self_signal", False))

        # --- world->map (ajuste se seu mapa tiver outro origin/resolution) ---
        self.resolution = float(rospy.get_param("~resolution", 0.16))
        self.origin_x = float(rospy.get_param("~origin_x", -8.0))
        self.origin_y = float(rospy.get_param("~origin_y", -8.0))

        # --- ambiente/vis ---
        self.env = Environment(width=self.width, height=self.height)
        self.vis = VisualizationSystem(self.env, agents_ref=[], robot_name=self.robot_name, view_mode='trail')

        self.tau_map = np.zeros((self.height, self.width), dtype=np.float32)

        # --- poses (somente se use_pose_sensor=True; NÃO entram na decisão) ---
        self.my_pose = None
        self.other_poses = {}

        self.other_robots = [f"robot_{i}" for i in range(self.num_robots_total) if f"robot_{i}" != self.robot_name]

        # --- ROS ---
        self.map_ready = False
        rospy.Subscriber("/map", OccupancyGrid, self.map_cb, queue_size=1)
        rospy.Subscriber(f"/{self.robot_name}/odom", Odometry, self.odom_cb, queue_size=1)

        if self.use_pose_sensor:
            for rid in self.other_robots:
                rospy.Subscriber(f"/{rid}/odom", Odometry, self._mk_other_cb(rid), queue_size=1)

        self.conn_pub = rospy.Publisher(f"/{self.robot_name}/physarum/connections", String, queue_size=10)

        self.log = open(f"/tmp/{self.robot_name}_aco_field_log.csv", "w")
        self.log.write("step,ants,success,best_dest,best_W,peak_thr\n")

    # ---------- callbacks / map ----------
    def map_cb(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution if msg.info.resolution > 0 else self.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        data = np.array(msg.data).reshape((height, width))
        data = np.flipud(data)

        for y in range(height):
            for x in range(width):
                if data[y, x] >= 50:
                    map_x = ((x * resolution + origin_x + 8) / 16.0 * self.width)
                    map_y = ((y * resolution + origin_y + 8) / 16.0 * self.height)
                    if 0 <= map_x < self.width and 0 <= map_y < self.height:
                        self.env.add_obstacle(map_y, map_x, size=1)

        self.map_ready = True

    def odom_cb(self, msg):
        self.my_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _mk_other_cb(self, rid):
        def cb(msg):
            self.other_poses[rid] = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        return cb

    # ---------- coord ----------
    def world_to_map(self, x, y):
        mx = int((x - self.origin_x) / self.resolution)
        my = self.height - int((y - self.origin_y) / self.resolution)
        return mx, my

    # ---------- sinal (sensor) -> food_map ----------
    def update_signal_maps_sensor(self):
        """Atualiza env.food_map / env.food_maps_by_robot como 'sensor' do simulador.
        O ACO usa só os mapas.
        """
        self.env.food_maps_by_robot.clear()
        self.env.food_sources.clear()

        if self.use_pose_sensor:
            for rid, (ox, oy) in self.other_poses.items():
                mx, my = self.world_to_map(ox, oy)
                self.env.food_sources.append((my, mx))
                self.env.add_robot_signal(rid, my, mx, intensity=self.comm_intensity, spread=self.signal_spread)

            if self.include_self_signal and self.my_pose is not None:
                mx, my = self.world_to_map(*self.my_pose)
                self.env.food_sources.append((my, mx))
                self.env.add_robot_signal(self.robot_name, my, mx, intensity=self.comm_intensity, spread=self.signal_spread)

        self.env.update_food()

    # ---------- heurística ----------
    def signal_at(self, x, y):
        if 0 <= x < self.width and 0 <= y < self.height:
            return float(self.env.food_map[y, x])
        return 0.0

    def is_obstacle(self, x, y):
        if 0 <= x < self.width and 0 <= y < self.height:
            return bool(self.env.obstacle_map[y, x] > 0.0)
        return True

    def neighbors8(self, x, y):
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.width and 0 <= ny < self.height and not self.is_obstacle(nx, ny):
                    yield nx, ny

    def is_peak(self, x, y):
        s = self.signal_at(x, y)
        if s < self.peak_threshold:
            return False
        for nx, ny in self.neighbors8(x, y):
            if self.signal_at(nx, ny) > s - self.peak_margin:
                return False
        return True

    # ---------- dinâmica do feromônio ----------
    def evaporate(self):
        self.tau_map *= (1.0 - self.rho)

    def diffuse_tau(self):
        if self.diffuse <= 0.0:
            return
        t = self.tau_map
        up = np.roll(t, -1, axis=0)
        dn = np.roll(t,  1, axis=0)
        lf = np.roll(t, -1, axis=1)
        rt = np.roll(t,  1, axis=1)
        avg = (t + up + dn + lf + rt) / 5.0
        self.tau_map = (1.0 - self.diffuse) * t + self.diffuse * avg
        self.tau_map[self.env.obstacle_map > 0] = 0.0

    def deposit_path(self, path, score):
        if not path:
            return
        delta = self.q * (0.2 + score)
        for (x, y) in path:
            if 0 <= x < self.width and 0 <= y < self.height and not self.is_obstacle(x, y):
                self.tau_map[y, x] += delta

    # ---------- escolha do passo ----------
    def choose_step(self, x, y, sx, sy, visited):
        candidates = list(self.neighbors8(x, y))
        if not candidates:
            return None

        s0 = self.signal_at(x, y)

        # viés para explorar "pra fora" do ponto inicial, até achar gradiente útil
        # (importante quando o sinal longe é fraco e o gradiente local é quase plano)
        maxdist = math.hypot(self.width, self.height) + 1e-9

        weights = []
        for nx, ny in candidates:
            tau = float(self.tau_map[ny, nx]) + 1e-9
            s1 = self.signal_at(nx, ny)

            # heurística principal: intensidade + subida de gradiente (bem mais forte que antes)
            grad_up = max(0.0, s1 - s0)
            eta = max(1e-9, 0.5 * s1 + 0.5 * grad_up)

            # penaliza revisitar células (evita ficar "rodando" perto do spawn)
            if (nx, ny) in visited:
                eta *= 0.15

            # empurra levemente para longe do spawn (só para quebrar simetria e sair da região local)
            d_out = math.hypot(nx - sx, ny - sy) / maxdist
            eta *= (1.0 + 0.8 * d_out)

            w = (tau ** self.alpha) * (eta ** self.beta)
            weights.append(w)

        total = float(sum(weights))
        if total <= 0.0:
            return candidates[np.random.randint(len(candidates))]

        r = np.random.rand() * total
        acc = 0.0
        for (nx, ny), w in zip(candidates, weights):
            acc += w
            if acc >= r:
                return (nx, ny)
        return candidates[-1]

    # ---------- extração de conexões ----------
    def best_connection(self):
        """Escolhe robô j maximizando W_j = sum(tau_map * S_j)."""
        if not self.env.food_maps_by_robot:
            return None, 0.0
        t = self.tau_map
        best_id, best_w = None, -1.0
        for rid, s_map in self.env.food_maps_by_robot.items():
            w = float(np.sum(t * s_map))
            if w > best_w:
                best_w = w
                best_id = rid
        return best_id, best_w

    # ---------- loop ----------
    def step(self, step):
        if self.use_pose_sensor:
            self.update_signal_maps_sensor()

        if self.my_pose is None:
            return

        sx, sy = self.world_to_map(*self.my_pose)
        sx = int(np.clip(sx, 0, self.width - 1))
        sy = int(np.clip(sy, 0, self.height - 1))

        if self.is_obstacle(sx, sy):
            # tenta achar vizinho livre
            ok = False
            for nx, ny in self.neighbors8(sx, sy):
                sx, sy = nx, ny
                ok = True
                break
            if not ok:
                return

        # dinâmica tau
        self.evaporate()
        self.diffuse_tau()

        success = 0
        ants = [FieldAnt(sx, sy) for _ in range(self.ants_per_cycle)]

        for ant in ants:
            max_s = self.signal_at(ant.x, ant.y)
            for _ in range(self.max_steps):
                ant.steps += 1

                if self.is_peak(ant.x, ant.y):
                    s_end = self.signal_at(ant.x, ant.y)
                    score = float(s_end / (1.0 + 0.01 * len(ant.path)))
                    self.deposit_path(ant.path, score)
                    success += 1
                    break

                nxt = self.choose_step(ant.x, ant.y, sx, sy, ant.visited)
                if nxt is None:
                    break
                ant.x, ant.y = nxt
                ant.path.append(nxt)
                ant.visited.add(nxt)
                s_now = self.signal_at(ant.x, ant.y)
                if s_now > max_s:
                    max_s = s_now

            else:
                # não encontrou pico dentro do limite
                if self.deposit_on_fail and len(ant.path) > 2:
                    score = float(self.fail_scale * max_s / (1.0 + 0.01 * len(ant.path)))
                    self.deposit_path(ant.path, score)

        # envia tau para visualização (trail_map)
        self.env.trail_map[:, :] = self.tau_map
        self.env.stable_trail_map = self.env.trail_map.copy()

        # escolhe conexão (por W_j)
        best_id, best_w = self.best_connection()
        self.conn_pub.publish(String(data=best_id if best_id else ""))
       
        #ants_viz = [VizAnt(a.y, a.x) for a in ants]
        #self.vis.update(agents=ants_viz, step=step)

        if step % 10 == 0:
            self.log.write(f"{step},{self.ants_per_cycle},{success},{best_id if best_id else ''},{best_w:.6f},{self.peak_threshold:.3f}\n")
            self.log.flush()

    def run(self):
        while not rospy.is_shutdown() and not self.map_ready:
            rospy.sleep(0.1)

        rate = rospy.Rate(int(rospy.get_param("~rate_hz", 10)))
        step = 0
        while not rospy.is_shutdown():
            self.step(step)
            step += 1
            rate.sleep()


if __name__ == "__main__":
    try:
        AcoFieldNode().run()
    except rospy.ROSInterruptException:
        pass