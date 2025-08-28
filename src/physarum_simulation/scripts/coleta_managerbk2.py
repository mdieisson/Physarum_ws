#!/usr/bin/env python3
import rospy
from physarum_simulation.msg import ColetaEvent
import time

# Mantém um log de eventos e estado atual dos pontos
event_log = {}  # chave = (x,y,timestamp,robot_id), valor = ColetaEvent
pontos_ocupados = {}  # chave = (x,y), valor = {"status": str, "robot_id": str, "timestamp": float}
robot_name = rospy.get_param("~robot_name", "robot_2")
# Recebe novos eventos e mescla no log
def event_callback(msg):
    global event_log, pontos_ocupados

    # Cria chave única para evitar duplicação (cada evento é único pelo timestamp+robot_id)
    key = (msg.x, msg.y, msg.timestamp, msg.robot_id)
    if key not in event_log:
        event_log[key] = msg
        rospy.loginfo(f"[{robot_name}] Recebeu evento {msg.action} em ({msg.x:.1f},{msg.y:.1f}) por {msg.robot_id}")
        # Recalcula estado após novo evento
        atualizar_estado()

# Atualiza estado dos pontos aplicando todos os eventos em ordem de timestamp
def atualizar_estado():
    global pontos_ocupados
    pontos_ocupados.clear()

    # Ordena eventos por timestamp
    eventos_ordenados = sorted(event_log.values(), key=lambda e: e.timestamp)
    for ev in eventos_ordenados:
        chave = (ev.x, ev.y)
        pontos_ocupados[chave] = {
            "status": ev.action,  # "ocupar" ou "liberar"
            "robot_id": ev.robot_id,
            "timestamp": ev.timestamp
        }

# Publica todos os eventos conhecidos (para sincronização ao entrar na rede)
def publicar_log_completo(pub):
    for ev in event_log.values():
        pub.publish(ev)
    rospy.loginfo(f"[{robot_name}] Reenviou {len(event_log)} eventos para sincronização.")

# Cria e publica um novo evento
def registrar_evento(pub, x, y, action):
    ev = ColetaEvent()
    ev.robot_id = robot_name
    ev.x = x
    ev.y = y
    ev.action = action  # "ocupar" ou "liberar"
    ev.timestamp = rospy.get_time()
    # Adiciona ao log e publica
    key = (ev.x, ev.y, ev.timestamp, ev.robot_id)
    event_log[key] = ev
    pub.publish(ev)
    rospy.loginfo(f"[{robot_name}] Registrou evento {action} para ponto ({x:.1f},{y:.1f})")
    atualizar_estado()

if __name__ == "__main__":
    rospy.init_node("coleta_manager", anonymous=True)
    robot_name = rospy.get_param("~robot_name", "robot_2")

    rospy.Subscriber("/coleta_events", ColetaEvent, event_callback)
    pub = rospy.Publisher("/coleta_events", ColetaEvent, queue_size=10)

    rate = rospy.Rate(0.5)  # Publica log a cada 2s para sincronização
    time.sleep(2)
    rospy.loginfo(f"[{robot_name}] Gerenciador de eventos iniciado.")

    while not rospy.is_shutdown():
        # Periodicamente reenvia o log completo para novos robôs que entrarem
        publicar_log_completo(pub)
        rate.sleep()
