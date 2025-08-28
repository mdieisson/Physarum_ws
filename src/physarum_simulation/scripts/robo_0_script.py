#!/usr/bin/env python3
import rospy
import math
import actionlib
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from stage_ros.srv import SetModelPose
from std_msgs.msg import String
from threading import Lock

# ========== CONFIG ==========
robot_name = 'robot_0'
pontos_coleta = [(-4, 4), (0, 4), (4, 4)]
ponto_final = (0.0, 0.0)

# ========== VARIÁVEIS ==========
robot_pose = None
container_acoplado = False
fase = 'inicio'
lista_containers = []
lock = Lock()

# ========== CALLBACKS ==========
def odom_callback(msg):
    global robot_pose
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
    theta = math.atan2(siny_cosp, cosy_cosp)
    robot_pose = (x, y, theta)

def containers_callback(msg):
    global lista_containers
    with lock:
        lista_containers = [c.strip() for c in msg.data.split(',') if c.strip()]

# ========== FUNÇÕES AUXILIARES ==========
def euclidean(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def enviar_meta(client, x, y):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0
    client.send_goal(goal)

def aguardar_chegada(client, destino, timeout=120.0):
    success = client.wait_for_result(rospy.Duration(timeout))
    if not success:
        client.cancel_goal()
        rospy.logwarn(f"{robot_name}: Timeout ao tentar alcançar {destino}")
        return False

    state = client.get_state()
    if state == GoalStatus.SUCCEEDED:
        rospy.loginfo(f"{robot_name}: Chegou em {destino}")
        return True
    else:
        rospy.logwarn(f"{robot_name}: Falha ao alcançar {destino} - Status: {state}")
        return False

def teletransportar_container(set_pose_proxy, base_pose, container_name, offset=-0.5):
    x, y, theta = base_pose
    req_x = x + offset * math.cos(theta)
    req_y = y + offset * math.sin(theta)
    req_a = theta
    try:
        resp = set_pose_proxy(model_name=container_name, x=req_x, y=req_y, a=req_a)
        if not resp.success:
            rospy.logwarn(f"Falha ao mover {container_name}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Erro ao chamar serviço set_model_pose: {e}")

# ========== MAIN ==========
def main():
    global robot_pose, container_acoplado, fase

    rospy.init_node("controller_multirobot_com_lista")
    rospy.Subscriber(f"/{robot_name}/odom", Odometry, odom_callback)
    rospy.Subscriber("/lista_containers", String, containers_callback)

    rospy.wait_for_service("/set_model_pose")
    set_pose_proxy = rospy.ServiceProxy("/set_model_pose", SetModelPose)

    client = actionlib.SimpleActionClient(f"/{robot_name}/move_base", MoveBaseAction)
    rospy.loginfo(f"{robot_name}: Aguardando move_base...")
    client.wait_for_server()
    rospy.loginfo(f"{robot_name}: Conectado ao move_base.")

    rate = rospy.Rate(10)
    rospy.loginfo("Aguardando pose inicial do robô...")
    while not rospy.is_shutdown() and robot_pose is None:
        rate.sleep()

    while not rospy.is_shutdown():
        with lock:
            if not lista_containers:
                rate.sleep()
                continue
            container_name = lista_containers.pop(0)

        # Vai até o ponto de coleta mais próximo
        ponto = min(pontos_coleta, key=lambda p: euclidean(robot_pose[:2], p))
        rospy.loginfo(f"{robot_name}: indo buscar {container_name} no ponto {ponto}")
        fase = 'indo_para_coleta'
        enviar_meta(client, *ponto)
        if not aguardar_chegada(client, ponto):
            continue

        rospy.loginfo(f"{robot_name}: coletando container {container_name}")
        container_acoplado = True
        fase = 'indo_para_destino'
        rospy.loginfo(f"{robot_name}: indo para ponto final {ponto_final}")
        enviar_meta(client, *ponto_final)
        if aguardar_chegada(client, ponto_final):
            teletransportar_container(set_pose_proxy, robot_pose, container_name)
            rospy.loginfo(f"{robot_name}: entregou {container_name}")
            container_acoplado = False

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
