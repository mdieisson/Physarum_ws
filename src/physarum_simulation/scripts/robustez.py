#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from nav_msgs.msg import Odometry
from std_msgs.msg import String

def parse_peers(s):
    # aceita "robot_1, robot_2" ou "robot_1 robot_2" etc.
    s = (s or "").strip()
    if not s:
        return set()
    s = s.replace(",", " ")
    parts = [p.strip() for p in s.split() if p.strip()]
    return set(parts)

class MonitorConectividade:
    def __init__(self):
        rospy.init_node("monitor_conectividade_global")

        # Pode passar a lista de robôs por parâmetro ROS: ~robot_ids: ["robot_0", "robot_1", ...]
        self.robot_ids = rospy.get_param("~robot_ids", ["robot_0", "robot_1", "robot_2"])
        self.start_time = time.time()

        # Estado por robô
        self.posicoes_iniciais = {}     # rid -> (x, y)
        self.peers_atuais = {}          # rid -> set(peers)
        self.peers_anteriores = {}      # rid -> set(peers) anterior para detectar mudanças
        self.perda_desde = {}           # rid -> timestamp quando ficou sem pares (lost all)
        self.ja_conectou = set()        # rids que já tiveram qualquer conexão não-vazia

        # Subs
        for rid in self.robot_ids:
            rospy.Subscriber(f"/{rid}/odom", Odometry, self._mk_odom_cb(rid), queue_size=10)
            rospy.Subscriber(f"/{rid}/physarum/connections", String, self._mk_conn_cb(rid), queue_size=100)

        # Relatório periódico no terminal
        periodo = rospy.get_param("~periodo_resumo_s", 0.001)
        self._timer = rospy.Timer(rospy.Duration(periodo), self._resumo)

        rospy.loginfo("Monitor de conectividade iniciado para: %s", ", ".join(self.robot_ids))

    # --- Callbacks ---

    def _mk_odom_cb(self, rid):
        def cb(msg):
            if rid not in self.posicoes_iniciais:
                p = msg.pose.pose.position
                self.posicoes_iniciais[rid] = (round(p.x, 2), round(p.y, 2))
                rospy.loginfo("[%.2fs] %s nasceu em (%.2f, %.2f)",
                              self._t(), rid, p.x, p.y)
        return cb

    def _mk_conn_cb(self, rid):
        def cb(msg):
            now = time.time()
            novos = parse_peers(msg.data)

            ant = self.peers_atuais.get(rid, set())
            if novos == ant:
                return  # nada mudou

            self.peers_anteriores[rid] = ant
            self.peers_atuais[rid] = novos

            # Eventos
            if (not ant) and novos:
                # Entrou em conectividade (primeira ou reconexão)
                if rid not in self.ja_conectou:
                    self.ja_conectou.add(rid)
                    rospy.loginfo("[%.4fs] %s CONECTOU pela primeira vez: %s",
                                  self._t(), rid, self._fmt(novos))
                else:
                    # Reconexão após ter ficado isolado
                    t_perda = self.perda_desde.get(rid, now)
                    dt = now - t_perda
                    rospy.loginfo("[%.4fs] %s RECONEXÃO após %.2fs: %s",
                                  self._t(), rid, dt, self._fmt(novos))
                # limpamos o marcador de perda
                self.perda_desde.pop(rid, None)

            elif ant and (not novos):
                # Perdeu todas as conexões
                self.perda_desde[rid] = now
                rospy.logwarn("[%.4fs] %s PERDEU TODAS AS CONEXÕES (antes: %s)",
                              self._t(), rid, self._fmt(ant))

            elif ant and novos:
                # Mudou o conjunto de pares (substituições, remoções parciais, etc.)
                perdidos = ant - novos
                ganhos = novos - ant
                if perdidos:
                    rospy.logwarn("[%.4fs] %s perdeu pares: %s (agora: %s)",
                                  self._t(), rid, self._fmt(perdidos), self._fmt(novos))
                if ganhos:
                    rospy.loginfo("[%.4fs] %s ganhou pares: %s (agora: %s)",
                                  self._t(), rid, self._fmt(ganhos), self._fmt(novos))
        return cb

    # --- Utilidades ---

    def _fmt(self, s):
        if not s:
            return "∅"
        return ", ".join(sorted(s))

    def _t(self):
        return time.time() - self.start_time

    def _resumo(self, _evt):
        linhas = []
        linhas.append("----- RESUMO CONECTIVIDADE (t=%.4fs) -----" % self._t())
        for rid in self.robot_ids:
            pos = self.posicoes_iniciais.get(rid, ("-", "-"))
            peers = self.peers_atuais.get(rid, set())
            status = "ISOLADO" if not peers else "OK"
            if rid in self.perda_desde:
                dt = time.time() - self.perda_desde[rid]
                status += " (isolado há %.2fs)" % dt
            linhas.append("  %-8s pos=%s  pares={%s}  %s" %
                          (rid, pos, self._fmt(peers), status))
        rospy.loginfo("\n" + "\n".join(linhas))

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        MonitorConectividade().spin()
    except rospy.ROSInterruptException:
        pass
