#!/usr/bin/env python3
import rospy
import math
import actionlib
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from stage_ros.srv import SetModelPose, GetModelPose
from physarum_simulation.msg import ContainerTask

robot_pose = None
tarefa_atual = None
container_acoplado = False
pontos_coleta = [(-4, 4.0), (0, 4.0), (4, 4.0)]
ponto_entrega = (0.0, -6.5)
posicoes_estoque = [(-6, -8), (-2, -8), (2, -8), (6, -8)]
estoque_ocupado = set()
pontos_ocupados = {}

def odom_callback(msg):
    global robot_pose
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
    theta = math.atan2(siny_cosp, cosy_cosp)
    robot_pose = (x, y, theta)

def task_callback(msg):
    global tarefa_atual
    # Sempre mantém a task atualizada
    if tarefa_atual and msg.id == tarefa_atual.id:
        tarefa_atual = msg
    elif msg.robot_id == robot_name and msg.status == "em_execucao":
        tarefa_atual = msg

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
        return False
    return client.get_state() == GoalStatus.SUCCEEDED

def teletransportar_container(set_pose_proxy, base_pose, container_name, offset=-0.6):
    x, y, theta = base_pose
    req_x = x + offset * math.cos(theta)
    req_y = y + offset * math.sin(theta)
    req_a = theta
    try:
        resp = set_pose_proxy(model_name=container_name, x=req_x, y=req_y, a=req_a)
        return resp.success
    except rospy.ServiceException:
        return False

def container_existe(container_id):
    try:
        rospy.wait_for_service("/get_model_pose", timeout=1.0)
        get_pose = rospy.ServiceProxy("/get_model_pose", GetModelPose)
        resp = get_pose(container_id)
        return resp is not None
    except rospy.ServiceException:
        return False

def escolher_posicao_estoque():
    for pos in posicoes_estoque:
        if pos not in estoque_ocupado:
            estoque_ocupado.add(pos)
            return pos
    return None

def escolher_ponto_coleta_disponivel():
    livres = [p for p in pontos_coleta if pontos_ocupados.get(p) in [None, robot_name]]
    if not livres:
        return None
    return min(livres, key=lambda p: euclidean(robot_pose[:2], p))

def publicar_entrega(pub, task):
    task.status = "entregue"
    task.timestamp = rospy.get_time()
    pub.publish(task)

if __name__ == "__main__":
    rospy.init_node("controller_execucao", anonymous=True)
    robot_name = rospy.get_param("~robot_name", "robot_2")

    rospy.Subscriber(f"/{robot_name}/odom", Odometry, odom_callback)
    rospy.Subscriber("/task_updates", ContainerTask, task_callback)
    pub = rospy.Publisher("/task_updates", ContainerTask, queue_size=10)

    rospy.wait_for_service("/set_model_pose")
    set_pose_proxy = rospy.ServiceProxy("/set_model_pose", SetModelPose)

    client = actionlib.SimpleActionClient(f"/{robot_name}/move_base", MoveBaseAction)
    client.wait_for_server()

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        if tarefa_atual is None:
            rate.sleep()
            continue

        # Verifica se container ainda existe
        if not container_existe(tarefa_atual.id):
            tarefa_atual.status = "removida"
            tarefa_atual.robot_id = ""
            pub.publish(tarefa_atual)
            tarefa_atual = None
            rate.sleep()
            continue

        # Vai até ponto de coleta
        ponto = escolher_ponto_coleta_disponivel()
        if not ponto:
            rate.sleep()
            continue
        pontos_ocupados[ponto] = robot_name
        enviar_meta(client, *ponto)
        if not aguardar_chegada(client, ponto):
            tarefa_atual = None
            del pontos_ocupados[ponto]
            continue

        # Antes de acoplar, verifica se ainda é dono e container existe
        if tarefa_atual.robot_id != robot_name or not container_existe(tarefa_atual.id):
            del pontos_ocupados[ponto]
            tarefa_atual = None
            rate.sleep()
            continue

        # Acopla e move até entrega
        if robot_pose and teletransportar_container(set_pose_proxy, robot_pose, tarefa_atual.id):
            container_acoplado = True
            enviar_meta(client, *ponto_entrega)
            while not rospy.is_shutdown():
                estado = client.get_state()
                if estado == GoalStatus.SUCCEEDED:
                    break
                if estado in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
                    break
                # Teletransporta só se ainda for dono
                if container_acoplado and robot_pose and tarefa_atual and tarefa_atual.robot_id == robot_name:
                    teletransportar_container(set_pose_proxy, robot_pose, tarefa_atual.id)
                rate.sleep()

            destino = escolher_posicao_estoque()
            if destino:
                set_pose_proxy(tarefa_atual.id, destino[0], destino[1], 0.0)
            publicar_entrega(pub, tarefa_atual)

        if ponto in pontos_ocupados:
            del pontos_ocupados[ponto]

        tarefa_atual = None
        container_acoplado = False
        rate.sleep()