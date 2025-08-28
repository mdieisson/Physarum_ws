#!/usr/bin/env python3
import rospy
from physarum_simulation.msg import ColetaEvent
from std_msgs.msg import String
import time

# --- ESTADOS GLOBAIS ---
event_log = {}  # chave = (x,y,timestamp,robot_id) -> evento
pontos_ocupados = {}  # chave = (x,y) -> {"status": str, "robot_id": str, "timestamp": float}
physarum_neighbors = set()
robot_name = None

# --- CALLBACKS ---
def event_callback(msg):
    global event_log, pontos_ocupados
    key = (msg.x, msg.y, msg.timestamp, msg.robot_id)
    if key not in event_log:
        event_log[key] = msg
        atualizar_estado()
        rospy.loginfo(f"[{robot_name}] Evento {msg.action} em ({msg.x:.1f},{msg.y:.1f}) por {msg.robot_id}")

def physarum_callback(msg):
    global physarum_neighbors
    physarum_neighbors.clear()
    for conn in msg.data.split(','):
        if f"{robot_name}<->" in conn or f"<->{robot_name}" in conn:
            parts = conn.split('<->')
            physarum_neighbors.add(parts[0] if parts[1] == robot_name else parts[1])

# --- FUNÇÕES ---
def atualizar_estado():
    global pontos_ocupados
    pontos_ocupados.clear()
    for ev in sorted(event_log.values(), key=lambda e: e.timestamp):
        pontos_ocupados[(ev.x, ev.y)] = {"status": ev.action, "robot_id": ev.robot_id, "timestamp": ev.timestamp}

def publicar_log_completo(pub):
    for ev in event_log.values():
        pub.publish(ev)
    rospy.loginfo(f"[{robot_name}] Reenviou {len(event_log)} eventos para sincronização.")

def registrar_evento(pub, x, y, action):
    ev = ColetaEvent()
    ev.robot_id = str(robot_name) if robot_name else "unassigned"
    ev.x = x
    ev.y = y
    ev.action = str(action)
    ev.timestamp = rospy.get_time()
    event_log[(ev.x, ev.y, ev.timestamp, ev.robot_id)] = ev
    pub.publish(ev)
    atualizar_estado()
    rospy.loginfo(f"[{robot_name}] Registrou evento {action} para ponto ({x:.1f},{y:.1f})")

def get_pontos_ocupados():
    return pontos_ocupados.copy()

# --- LOOP PRINCIPAL ---
if __name__ == "__main__":
    rospy.init_node("coleta_manager", anonymous=True)
    robot_name = rospy.get_param("~robot_name", "robot_2")

    rospy.Subscriber("/coleta_events", ColetaEvent, event_callback)
    rospy.Subscriber("/physarum/connections", String, physarum_callback)

    pub = rospy.Publisher("/coleta_events", ColetaEvent, queue_size=10)

    rate = rospy.Rate(0.5)
    time.sleep(2)
    rospy.loginfo(f"[{robot_name}] Gerenciador de eventos iniciado e integrado.")

    while not rospy.is_shutdown():
        if physarum_neighbors:
            publicar_log_completo(pub)
        rate.sleep()
