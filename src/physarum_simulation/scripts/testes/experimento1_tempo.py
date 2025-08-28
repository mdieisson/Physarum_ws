#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from math import sqrt
from collections import defaultdict
import csv
import matplotlib.pyplot as plt
import os
import threading


#_sim_id:=1 _tempo_limite:=300
# === CONFIGURAÇÕES ===
robot_names = ["robot_0", "robot_1", "robot_2"]
connection_topics = {name: f"/{name}/physarum/connections" for name in robot_names}
odom_topics = {name: f"/{name}/odom" for name in robot_names}
output_folder = "logs_physarum"

# === ROS PARAMS ===
sim_id = rospy.get_param("~sim_id", 1)
tempo_limite = rospy.get_param("~tempo_limite", 300)  # segundos

# Arquivos com ID da simulação
csv_filename = os.path.join(output_folder, f"sim_{sim_id:03d}.csv")
hist_dist_file = os.path.join(output_folder, f"hist_dist_sim_{sim_id:03d}.png")
hist_dur_file = os.path.join(output_folder, f"hist_dur_sim_{sim_id:03d}.png")
disp_file = os.path.join(output_folder, f"dispersao_sim_{sim_id:03d}.png")

# === ESTRUTURAS GLOBAIS ===
last_connected = {}
robot_poses = {}
active_connections = {}
connection_durations = defaultdict(list)

def normalizar_par(r1, r2):
    return tuple(sorted([r1, r2]))

def distancia(p1, p2):
    if p1 is None or p2 is None:
        return None
    return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def verificar_conexoes_bilaterais(current_time):
    pares_atualmente_conectados = set()
    for r1 in robot_names:
        destino = last_connected.get(r1, None)
        if destino and last_connected.get(destino, None) == r1:
            par = normalizar_par(r1, destino)
            pares_atualmente_conectados.add(par)

    pares_anteriores = set(active_connections.keys())

    for par in pares_atualmente_conectados - pares_anteriores:
        active_connections[par] = current_time
        pos1 = robot_poses.get(par[0])
        pos2 = robot_poses.get(par[1])
        d = distancia(pos1, pos2)
        if d is not None:
            connection_durations[par].append((d, 0.0))

    for par in pares_anteriores - pares_atualmente_conectados:
        dur = (current_time - active_connections[par]).to_sec()
        if connection_durations[par] and connection_durations[par][-1][1] == 0.0:
            dist, _ = connection_durations[par][-1]
            connection_durations[par][-1] = (dist, dur)
        del active_connections[par]

def make_connection_callback(robot_name):
    def callback(msg):
        current_time = rospy.Time.now()
        last_connected[robot_name] = msg.data.strip() if msg.data.strip() else None
        verificar_conexoes_bilaterais(current_time)
    return callback

def make_odom_callback(robot_name):
    def callback(msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        robot_poses[robot_name] = (x, y)
    return callback

def salvar_csv():
    os.makedirs(output_folder, exist_ok=True)
    with open(csv_filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Par de Robôs", "Distância (m)", "Duração (s)"])
        for par, valores in connection_durations.items():
            for dist, dur in valores:
                writer.writerow([f"{par[0]}<->{par[1]}", f"{dist:.2f}", f"{dur:.2f}"])
    rospy.loginfo(f"[Sim {sim_id}] CSV salvo: {csv_filename}")

def gerar_graficos():
    distancias = []
    duracoes = []
    for valores in connection_durations.values():
        for dist, dur in valores:
            if dist is not None and dur > 0:
                distancias.append(dist)
                duracoes.append(dur)

    if not distancias or not duracoes:
        rospy.logwarn("Sem dados suficientes para gerar gráficos.")
        return

    # Histograma de distâncias
    plt.figure()
    plt.hist(distancias, bins=10, edgecolor='black')
    plt.title(f"Sim {sim_id} - Histograma de Distâncias")
    plt.xlabel("Distância (m)")
    plt.ylabel("Frequência")
    plt.savefig(hist_dist_file)

    # Histograma de durações
    plt.figure()
    plt.hist(duracoes, bins=10, edgecolor='black')
    plt.title(f"Sim {sim_id} - Histograma de Durações")
    plt.xlabel("Duração (s)")
    plt.ylabel("Frequência")
    plt.savefig(hist_dur_file)

    # Dispersão
    plt.figure()
    plt.scatter(distancias, duracoes)
    plt.title(f"Sim {sim_id} - Distância × Duração")
    plt.xlabel("Distância (m)")
    plt.ylabel("Duração (s)")
    plt.grid(True)
    plt.savefig(disp_file)

    rospy.loginfo(f"[Sim {sim_id}] Gráficos salvos em: {output_folder}")

def shutdown_hook():
    rospy.loginfo(f"[Sim {sim_id}] Encerrando...")
    salvar_csv()
    gerar_graficos()

def finalizar_automaticamente():
    rospy.sleep(tempo_limite)
    rospy.loginfo(f"[Sim {sim_id}] Tempo limite alcançado ({tempo_limite}s). Encerrando simulação.")
    rospy.signal_shutdown("Fim automático da simulação.")

if __name__ == '__main__':
    rospy.init_node("monitoramento_conexoes_com_graficos")
    rospy.on_shutdown(shutdown_hook)

    for robot_name, topic in connection_topics.items():
        rospy.Subscriber(topic, String, make_connection_callback(robot_name))

    for robot_name, topic in odom_topics.items():
        rospy.Subscriber(topic, Odometry, make_odom_callback(robot_name))

    # Encerramento automático
    threading.Thread(target=finalizar_automaticamente).start()

    rospy.loginfo(f"[Sim {sim_id}] Monitoramento iniciado. Tempo limite: {tempo_limite}s")
    rospy.spin()

