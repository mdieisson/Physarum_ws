#!/usr/bin/env python3
import rospy
import time
import csv
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class ConectividadeGlobal:
    def __init__(self):
        rospy.init_node("monitor_conectividade_global")

        self.robot_ids = ["robot_0", "robot_1", "robot_2"]  # Ajuste se tiver mais
        self.start_time = time.time()

        self.posicoes_iniciais = {}
        self.conexoes = {}

        for rid in self.robot_ids:
            rospy.Subscriber(f"/{rid}/odom", Odometry, self.make_odom_callback(rid))
            rospy.Subscriber(f"/{rid}/physarum/connections", String, self.make_connection_callback(rid))

        self.dados_coletados = False

    def make_odom_callback(self, robot_id):
        def callback(msg):
            if robot_id not in self.posicoes_iniciais:
                pos = msg.pose.pose.position
                self.posicoes_iniciais[robot_id] = (round(pos.x, 2), round(pos.y, 2))
                rospy.loginfo(f"{robot_id} cesta na posicao {pos}")
        return callback

    def make_connection_callback(self, robot_id):
        def callback(msg):
            now = time.time()
            if robot_id not in self.conexoes and msg.data.strip():
                self.conexoes[robot_id] = {
                    "tempo": round(now - self.start_time, 2),
                    "conectado_com": msg.data.strip()
                }
                rospy.loginfo(f"{robot_id} conectou com {msg.data.strip()} em {now - self.start_time:.2f}s")
        return callback

    def salvar_csv(self):
        with open("conectividade_global.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Robo", "Posição Inicial (x,y)", "Robo Conectado", "Tempo para Conectar (s)"])
            for robot in self.robot_ids:
                pos = self.posicoes_iniciais.get(robot, ("-", "-"))
                conn = self.conexoes.get(robot, {})
                conectado_com = conn.get("conectado_com", "-")
                tempo = conn.get("tempo", "-")
                writer.writerow([robot, pos, conectado_com, tempo])

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if not self.dados_coletados and len(self.conexoes) == len(self.robot_ids):
                rospy.loginfo("Todos os robôs se conectaram. Salvando CSV.")
                self.salvar_csv()
                self.dados_coletados = True
            rate.sleep()

if __name__ == "__main__":
    try:
        cg = ConectividadeGlobal()
        cg.run()
    except rospy.ROSInterruptException:
        pass

