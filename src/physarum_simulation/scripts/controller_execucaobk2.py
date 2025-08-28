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


        #!/usr/bin/env python3
import rospy
import math
import json
import actionlib
import tf.transformations
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from stage_ros.srv import SetModelPose, GetModelPose
from physarum_simulation.msg import ContainerTask, ColetaEvent
from std_msgs.msg import String
from std_srvs.srv import Empty

# ==============================
# VARIÁVEIS GLOBAIS
# ==============================
robot_name = None
robot_pose = None
origem = (0.0, 0.0, 0.0)
tarefa_atual = None
container_acoplado = False

task_local = {}        # {id: ContainerTask}
task_global = {}       # {id: ContainerTask}
processed_tasks = set()

connected_robot = None
active_links = {}
physarum_neighbors = set()

verificar_container_ativo = True  # pode ser desativado via ROS param

# ==============================
# CALLBACKS
# ==============================
def odom_callback(msg):
    global robot_pose
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    siny = 2 * (q.w * q.z + q.x * q.y)
    cosy = 1 - 2 * (q.y ** 2 + q.z ** 2)
    theta = math.atan2(siny, cosy)
    robot_pose = (x, y, theta)

def task_local_callback(msg):
    global tarefa_atual, task_local
    task_local[msg.id] = msg
    if not tarefa_atual:
        tarefa_atual = msg
    publish_all_tasks()

def physarum_callback(msg):
    global connected_robot, physarum_neighbors
    physarum_neighbors.clear()
    if msg.data.strip():
        connected_robot = msg.data.strip()
        physarum_neighbors.add(connected_robot)
    else:
        connected_robot = None
    update_links()
    publish_all_tasks()

def task_sync_callback(msg, origin):
    global processed_tasks, task_global, task_local, tarefa_atual
    try:
        data = json.loads(msg.data)
    except Exception:
        return
    tid = str(data.get("id", ""))
    if tid in processed_tasks:
        return
    processed_tasks.add(tid)
    if tid not in task_local:
        t = ContainerTask()
        t.id = tid
        t.coleta_x, t.coleta_y = data["coleta"]
        t.dest_x, t.dest_y = data["destino"]
        t.status = data["status"]
        t.robot_id = data.get("robot_id", "")
        task_local[tid] = t
    else:
        t = task_local[tid]
        t.status = data["status"]
        t.robot_id = data.get("robot_id", "")
        if tarefa_atual and tarefa_atual.id == tid and t.robot_id != robot_name:
            tarefa_atual = None
    task_global[tid] = task_local[tid]
    for neighbor, pub in active_links.items():
        if neighbor != origin:
            pub.publish(msg)

# ==============================
# FUNÇÕES AUXILIARES
# ==============================
def update_links():
    global active_links, connected_robot
    for r in list(active_links):
        if r != connected_robot:
            active_links[r].unregister()
            del active_links[r]
    if connected_robot and connected_robot not in active_links:
        topic = f"/link_{'_'.join(sorted([robot_name, connected_robot]))}/task_sync"
        pub = rospy.Publisher(topic, String, queue_size=10)
        active_links[connected_robot] = pub
        rospy.Subscriber(topic, String, task_sync_callback, callback_args=connected_robot)
        publish_all_tasks()

def publish_all_tasks():
    for t in list(task_local.values()) + list(task_global.values()):
        #if t.status == "livre" 
        payload = {
            "id": t.id,
            "coleta": (t.coleta_x, t.coleta_y),
            "destino": (t.dest_x, t.dest_y),
            "status": t.status,
            "robot_id": t.robot_id
        }
        msg = String()
        msg.data = json.dumps(payload)
        processed_tasks.add(t.id)
        for pub in active_links.values():
            pub.publish(msg)

def publish_task(task: ContainerTask):
    payload = {
        "id": task.id,
        "coleta": (task.coleta_x, task.coleta_y),
        "destino": (task.dest_x, task.dest_y),
        "status": task.status,
        "robot_id": task.robot_id
    }
    msg = String()
    msg.data = json.dumps(payload)
    processed_tasks.add(task.id)
    for pub in active_links.values():
        pub.publish(msg)

def enviar_meta(client, x, y, angulo=0, offset=0.8):
    dx, dy = 0, offset
    if robot_pose:
        angle = robot_pose[2]
        dx = offset * math.cos(angle)
        dy = offset * math.sin(angle)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x - dx
    goal.target_pose.pose.position.y = y - dy
    quat = tf.transformations.quaternion_from_euler(0, 0, math.radians(angulo))
    goal.target_pose.pose.orientation.x = quat[0]
    goal.target_pose.pose.orientation.y = quat[1]
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]
    client.send_goal(goal)

def aguardar_chegada(client, timeout=120.0):
    ok = client.wait_for_result(rospy.Duration(timeout))
    return ok and client.get_state() == GoalStatus.SUCCEEDED

def teletransportar(set_pose_proxy, pose, cid, offset_z=0.4):
    x, y, theta = pose
    return set_pose_proxy(model_name=cid, x=x, y=y, z=offset_z, a=theta).success

def container_existe(cid, posicao_esperada=None, tolerancia=0.5):
    """
    Verifica se o container existe e (opcionalmente) se está próximo da posição esperada.
    Se verificar_container_ativo for False, retorna True sempre (para testes).
    """
    if not verificar_container_ativo:
        return True
    try:
        resp = rospy.ServiceProxy("/get_model_pose", GetModelPose)(cid)
        if resp is None:
            return False
        if posicao_esperada:
            dx = resp.x - posicao_esperada[0]
            dy = resp.y - posicao_esperada[1]
            dist = (dx ** 2 + dy ** 2) ** 0.5
            return dist <= tolerancia
        return True
    except rospy.ServiceException:
        return False

def registrar_evento(pub, x, y, acao):
    ev = ColetaEvent()
    ev.robot_id = robot_name
    ev.x = x
    ev.y = y
    ev.action = acao
    ev.timestamp = rospy.get_time()
    pub.publish(ev)

def voltar_para_origem(client, tolerancia=0.2):
    if not robot_pose:
        return
    dist = math.hypot(robot_pose[0] - origem[0], robot_pose[1] - origem[1])
    if dist > tolerancia:
        enviar_meta(client, origem[0], origem[1], angulo=90, offset=0)
        aguardar_chegada(client, timeout=120)

# ==============================
# LOOP PRINCIPAL
# ==============================
if __name__ == "__main__":
    rospy.init_node("controller_execucao")
    robot_name = rospy.get_param("~robot_name", "robot_0")
    origem_param = rospy.get_param("~origem", [0.0, 0.0])
    origem = (float(origem_param[0]), float(origem_param[1]), 0.0)
    verificar_container_ativo = rospy.get_param("~verificar_container", True)

    clear_costmaps = rospy.ServiceProxy(f"/{robot_name}/move_base/clear_costmaps", Empty)
    rospy.Subscriber(f"/{robot_name}/odom", Odometry, odom_callback)
    rospy.Subscriber(f"/{robot_name}/task_local", ContainerTask, task_local_callback)
    rospy.Subscriber(f"/{robot_name}/physarum/connections", String, physarum_callback)

    pub_coleta = rospy.Publisher("/coleta_events", ColetaEvent, queue_size=10)
    set_pose_proxy = rospy.ServiceProxy("/set_model_pose", SetModelPose)
    client = actionlib.SimpleActionClient(f"/{robot_name}/move_base", MoveBaseAction)
    client.wait_for_server()

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        #publish_task(tarefa_atual)
        if not tarefa_atual:
            for tid, t in task_local.items():
                if t.status == "livre" and container_existe(t.id):
                    tarefa_atual = t
                    tarefa_atual.status = "em_execucao"
                    tarefa_atual.robot_id = robot_name
                    publish_task(tarefa_atual)
                    break

        if not tarefa_atual:
            voltar_para_origem(client)
            rate.sleep()
            continue

        ponto_coleta = (tarefa_atual.coleta_x, tarefa_atual.coleta_y)
        registrar_evento(pub_coleta, ponto_coleta[0], ponto_coleta[1], "ocupar")
        sucesso = False
        tentativas = 0
        while not rospy.is_shutdown() and not sucesso and tentativas < 2:
            enviar_meta(client, *ponto_coleta)
            sucesso= aguardar_chegada(client, timeout=120)
            if not sucesso:
                tentativas += 1
                rospy.logwarn(f"[{robot_name}] Falha ao chegar no ponto {ponto_coleta}, tentativa {tentativas}")
                try:
                    clear_costmaps()
                except rospy.ServiceException as e:
                    rospy.logerr(f"[{robot_name}] Falha ao limpar costmaps: {e}")
                rospy.sleep(1.0)

            if not sucesso:
                registrar_evento(pub_coleta, ponto_coleta[0], ponto_coleta[1], "liberar")
                tarefa_atual.status = "livre"
                tarefa_atual.robot_id = ""
                publish_task(tarefa_atual)
                tarefa_atual = None
                rate.sleep()
                continue
            if not container_existe(tarefa_atual.id):
                tarefa_atual.status = "removida"
                tarefa_atual.robot_id = ""
                publish_task(tarefa_atual)
                tarefa_atual = None
                continue

        if teletransportar(set_pose_proxy, robot_pose, tarefa_atual.id):
            container_acoplado = True
            destino = (tarefa_atual.dest_x, tarefa_atual.dest_y)
            enviar_meta(client, *destino, offset=0)
            while not rospy.is_shutdown() and client.get_state() not in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED]:
                if container_acoplado:
                    teletransportar(set_pose_proxy, robot_pose, tarefa_atual.id)
                rate.sleep()
            sucesso = (client.get_state() == GoalStatus.SUCCEEDED)
            if not sucesso:
                rospy.logwarn(f"[{robot_name}] Falha ao chegar ao destino {destino}, tentando novamente...")
                try:
                    clear_costmaps()
                except rospy.ServiceException as e:
                    rospy.logerr(f"[{robot_name}] Falha ao limpar costmaps: {e}")
                rospy.sleep(1.0)
            set_pose_proxy(tarefa_atual.id, -5.8, 1.0, 0.0, 0.0)
            tarefa_atual.status = "entregue"
            publish_task(tarefa_atual)

        registrar_evento(pub_coleta, ponto_coleta[0], ponto_coleta[1], "liberar")
        tarefa_atual = None
        container_acoplado = False
        publish_all_tasks()
        rate.sleep()

# #!/usr/bin/env python3
# import rospy
# import math
# import json
# import actionlib
# import tf.transformations
# from collections import defaultdict
# from nav_msgs.msg import Odometry
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from actionlib_msgs.msg import GoalStatus
# from stage_ros.srv import SetModelPose, GetModelPose
# from physarum_simulation.msg import ContainerTask, ColetaEvent
# from std_msgs.msg import String
# from std_srvs.srv import Empty

# # ==============================
# # VARIÁVEIS GLOBAIS
# # ==============================
# robot_name = None
# robot_pose = None
# origem = (0.0, 0.0, 0.0)
# tarefa_atual = None
# container_acoplado = False

# task_local = {}      # {id: ContainerTask}
# task_global = {}     # {id: ContainerTask}
# processed_tasks = set()  # IDs de tarefas já vistas

# connected_robot = None
# active_links = {}    # {robot_name: publisher}
# physarum_neighbors = set()

# # ==============================
# # CALLBACKS
# # ==============================
# def odom_callback(msg):
#     global robot_pose
#     x = msg.pose.pose.position.x
#     y = msg.pose.pose.position.y
#     q = msg.pose.pose.orientation
#     siny = 2 * (q.w * q.z + q.x * q.y)
#     cosy = 1 - 2 * (q.y ** 2 + q.z ** 2)
#     theta = math.atan2(siny, cosy)
#     robot_pose = (x, y, theta)

# def task_local_callback(msg):
#     global tarefa_atual, task_local
#     task_local[msg.id] = msg
#     if not tarefa_atual:
#         tarefa_atual = msg
#     publish_all_tasks()  # sempre redistribui quando há nova tarefa local

# def physarum_callback(msg):
#     """Atualiza vizinho conectado (apenas indicador visual) e dispara sincronização."""
#     global connected_robot, physarum_neighbors
#     physarum_neighbors.clear()
#     if msg.data.strip():
#         connected_robot = msg.data.strip()
#         physarum_neighbors.add(connected_robot)
#     else:
#         connected_robot = None
#     update_links()
#     publish_all_tasks()  # força atualização ao conectar/desconectar

# def task_sync_callback(msg, origin):
#     """Recebe tarefas de outro robô e propaga (gossip) + sincroniza."""
#     global processed_tasks, task_global, task_local, tarefa_atual
#     try:
#         data = json.loads(msg.data)
#     except Exception:
#         return

#     tid = str(data.get("id", ""))
#     if tid in processed_tasks:
#         return
#     processed_tasks.add(tid)

#     if tid not in task_local:
#         t = ContainerTask()
#         t.id = tid
#         t.coleta_x, t.coleta_y = data["coleta"]
#         t.dest_x, t.dest_y = data["destino"]
#         t.status = data["status"]
#         t.robot_id = data.get("robot_id", "")
#         task_local[tid] = t
#     else:
#         t = task_local[tid]
#         t.status = data["status"]
#         t.robot_id = data.get("robot_id", "")
#         if tarefa_atual and tarefa_atual.id == tid and t.robot_id != robot_name:
#             tarefa_atual = None

#     task_global[tid] = task_local[tid]
#     # Propaga para outros vizinhos (exceto origem)
#     for neighbor, pub in active_links.items():
#         if neighbor != origin:
#             pub.publish(msg)

# # ==============================
# # FUNÇÕES AUXILIARES
# # ==============================
# def update_links():
#     """Gerencia canais ponto-a-ponto de sincronização."""
#     global active_links, connected_robot
#     # Remove links obsoletos
#     for r in list(active_links):
#         if r != connected_robot:
#             active_links[r].unregister()
#             del active_links[r]
#     # Cria novo link se aplicável
#     if connected_robot and connected_robot not in active_links:
#         topic = f"/link_{'_'.join(sorted([robot_name, connected_robot]))}/task_sync"
#         pub = rospy.Publisher(topic, String, queue_size=10)
#         active_links[connected_robot] = pub
#         rospy.Subscriber(topic, String, task_sync_callback, callback_args=connected_robot)
#         # Envia tarefas conhecidas ao novo vizinho
#         publish_all_tasks()

# def publish_all_tasks():
#     """Envia estado completo de tarefas (locais + globais) para todos os vizinhos."""
#     for t in list(task_local.values()) + list(task_global.values()):
#         payload = {
#             "id": t.id,
#             "coleta": (t.coleta_x, t.coleta_y),
#             "destino": (t.dest_x, t.dest_y),
#             "status": t.status,
#             "robot_id": t.robot_id
#         }
#         msg = String()
#         msg.data = json.dumps(payload)
#         processed_tasks.add(t.id)
#         for pub in active_links.values():
#             pub.publish(msg)

# def publish_task(task: ContainerTask):
#     """Publica uma tarefa específica para vizinhos conectados."""
#     payload = {
#         "id": task.id,
#         "coleta": (task.coleta_x, task.coleta_y),
#         "destino": (task.dest_x, task.dest_y),
#         "status": task.status,
#         "robot_id": task.robot_id
#     }
#     msg = String()
#     msg.data = json.dumps(payload)
#     processed_tasks.add(task.id)
#     for pub in active_links.values():
#         pub.publish(msg)

# def enviar_meta(client, x, y, angulo=0, offset=0.8):
#     dx, dy = 0, offset
#     if robot_pose:
#         angle = robot_pose[2]
#         dx = offset * math.cos(angle)
#         dy = offset * math.sin(angle)
#     goal = MoveBaseGoal()
#     goal.target_pose.header.frame_id = "map"
#     goal.target_pose.header.stamp = rospy.Time.now()
#     goal.target_pose.pose.position.x = x - dx
#     goal.target_pose.pose.position.y = y - dy
#     yaw = math.radians(angulo)
#     quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
#     goal.target_pose.pose.orientation.x = quat[0]
#     goal.target_pose.pose.orientation.y = quat[1]
#     goal.target_pose.pose.orientation.z = quat[2]
#     goal.target_pose.pose.orientation.w = quat[3]
#     client.send_goal(goal)

# def aguardar_chegada(client, timeout=120.0):
#     ok = client.wait_for_result(rospy.Duration(timeout))
#     return ok and client.get_state() == GoalStatus.SUCCEEDED

# def teletransportar(set_pose_proxy, pose, cid, offset_z=0.4):
#     x, y, theta = pose
#     return set_pose_proxy(model_name=cid, x=x, y=y, z=offset_z, a=theta).success

# def container_existe(cid):
#     try:
#         resp = rospy.ServiceProxy("/get_model_pose", GetModelPose)(cid)
#         return resp is not None
#     except rospy.ServiceException:
#         return False

# def registrar_evento(pub, x, y, acao):
#     ev = ColetaEvent()
#     ev.robot_id = robot_name
#     ev.x = x
#     ev.y = y
#     ev.action = acao
#     ev.timestamp = rospy.get_time()
#     pub.publish(ev)

# def voltar_para_origem(client, tolerancia=0.2):
#     """Leva o robô de volta para a origem se não houver tarefas."""
#     if not robot_pose:
#         return
#     dist = math.hypot(robot_pose[0] - origem[0], robot_pose[1] - origem[1])
#     if dist <= tolerancia:
#         return
#     enviar_meta(client, origem[0], origem[1], angulo=90, offset=0)
#     aguardar_chegada(client, timeout=120)

# # ==============================
# # LOOP PRINCIPAL
# # ==============================
# if __name__ == "__main__":
#     rospy.init_node("controller_execucao")
#     robot_name = rospy.get_param("~robot_name", "robot_0")
#     origem_param = rospy.get_param("~origem", [0.0, 0.0])
#     origem = (float(origem_param[0]), float(origem_param[1]), 0.0)

#     clear_costmaps = rospy.ServiceProxy(f"/{robot_name}/move_base/clear_costmaps", Empty)
#     rospy.Subscriber(f"/{robot_name}/odom", Odometry, odom_callback)
#     rospy.Subscriber(f"/{robot_name}/task_local", ContainerTask, task_local_callback)
#     rospy.Subscriber(f"/{robot_name}/physarum/connections", String, physarum_callback)

#     pub_coleta = rospy.Publisher("/coleta_events", ColetaEvent, queue_size=10)
#     set_pose_proxy = rospy.ServiceProxy("/set_model_pose", SetModelPose)
#     client = actionlib.SimpleActionClient(f"/{robot_name}/move_base", MoveBaseAction)
#     client.wait_for_server()

#     rate = rospy.Rate(5)
#     while not rospy.is_shutdown():
#         # Se não há tarefa atual, tenta pegar uma livre
#         if not tarefa_atual:
#             for tid, t in task_local.items():
#                 if t.status == "livre" and container_existe(t.id):
#                     tarefa_atual = t
#                     tarefa_atual.status = "em_execucao"
#                     tarefa_atual.robot_id = robot_name
#                     publish_task(tarefa_atual)
#                     break

#         # Se ainda sem tarefa, volta à origem
#         if not tarefa_atual:
#             voltar_para_origem(client)
#             rate.sleep()
#             continue

#         # Coleta
#         ponto_coleta = (tarefa_atual.coleta_x, tarefa_atual.coleta_y)
#         registrar_evento(pub_coleta, ponto_coleta[0], ponto_coleta[1], "ocupar")
#         enviar_meta(client, *ponto_coleta)
#         if not aguardar_chegada(client, timeout=60) or not container_existe(tarefa_atual.id):
#             tarefa_atual.status = "livre"
#             tarefa_atual.robot_id = ""
#             publish_task(tarefa_atual)
#             tarefa_atual = None
#             rate.sleep()
#             continue

#         # Transporte e entrega
#         if teletransportar(set_pose_proxy, robot_pose, tarefa_atual.id):
#             container_acoplado = True
#             destino = (tarefa_atual.dest_x, tarefa_atual.dest_y)
#             enviar_meta(client, *destino, offset=0)
#             while not rospy.is_shutdown() and client.get_state() not in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED]:
#                 if container_acoplado:
#                     teletransportar(set_pose_proxy, robot_pose, tarefa_atual.id)
#                 rate.sleep()
#             # Container é descarregado no ponto fixo (-5.8, 1.0)
#             set_pose_proxy(tarefa_atual.id, -5.8, 1.0, 0.0, 0.0)
#             tarefa_atual.status = "entregue"
#             publish_task(tarefa_atual)

#         registrar_evento(pub_coleta, ponto_coleta[0], ponto_coleta[1], "liberar")
#         tarefa_atual = None
#         container_acoplado = False
#         publish_all_tasks()  # sincroniza após entrega
#         rate.sleep()


# #!/usr/bin/env python3
# import rospy
# import math
# import json
# import actionlib
# import tf.transformations
# from nav_msgs.msg import Odometry
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from actionlib_msgs.msg import GoalStatus
# from stage_ros.srv import SetModelPose, GetModelPose
# from physarum_simulation.msg import ContainerTask, ColetaEvent
# from std_msgs.msg import String
# from std_srvs.srv import Empty

# # ==============================
# # VARIÁVEIS GLOBAIS
# # ==============================
# robot_pose = None
# tarefa_atual = None
# container_acoplado = False
# physarum_neighbors = set()
# connected = False
# task_local = {}  # {id: ContainerTask}

# # ==============================
# # CALLBACKS
# # ==============================
# def odom_callback(msg):
#     global robot_pose
#     x = msg.pose.pose.position.x
#     y = msg.pose.pose.position.y
#     q = msg.pose.pose.orientation
#     siny = 2 * (q.w * q.z + q.x * q.y)
#     cosy = 1 - 2 * (q.y ** 2 + q.z ** 2)
#     theta = math.atan2(siny, cosy)
#     robot_pose = (x, y, theta)

# def task_local_callback(msg):
#     global tarefa_atual, task_local
#     task_local[msg.id] = msg
#     if not tarefa_atual:
#         tarefa_atual = msg

# def physarum_callback(msg):
#     global connected, physarum_neighbors
#     physarum_neighbors.clear()

#     # msg.data agora contém só os nomes dos robôs vizinhos, separados por vírgula
#     for neighbor in msg.data.split(','):
#         neighbor = neighbor.strip()
#         if neighbor and neighbor != robot_name:  # ignora vazio e o próprio robô
#             physarum_neighbors.add(neighbor)

#     # Está conectado se houver ao menos um vizinho
#     connected = len(physarum_neighbors) > 0

# def task_global_callback(msg):
#     """Sincroniza tarefas recebidas de outros robôs."""
#     global task_local, tarefa_atual
#     if not connected:
#         return
#     try:
#         data = json.loads(msg.data)
#         for tid, tinfo in data.items():
#             # Se já temos a tarefa localmente, atualizar status e dono
#             if tid in task_local:
#                 local_task = task_local[tid]
#                 local_task.status = tinfo["status"]
#                 local_task.robot_id = tinfo.get("robot_id", "")

#                 # Se outro robô assumiu a tarefa que este robô estava executando
#                 if tarefa_atual and tarefa_atual.id == tid and local_task.robot_id != robot_name:
#                     rospy.logwarn(f"[{robot_name}] Tarefa {tid} assumida por {local_task.robot_id}, liberando.")
#                     tarefa_atual = None
#                 continue

#             # Adicionar tarefa nova se livre
#             if tinfo["status"] == "livre":
#                 t = ContainerTask()
#                 t.id = tid
#                 t.coleta_x, t.coleta_y = tinfo["coleta"]
#                 t.dest_x, t.dest_y = tinfo["destino"]
#                 t.status = tinfo["status"]
#                 t.robot_id = tinfo.get("robot_id", "")
#                 task_local[tid] = t
#     except Exception as e:
#         rospy.logwarn(f"[{robot_name}] Erro ao processar task_global: {e}")

# # ==============================
# # FUNÇÕES AUXILIARES
# # ==============================
# def distance_to(point):
#     if not robot_pose:
#         return float('inf')
#     return math.hypot(robot_pose[0] - point[0], robot_pose[1] - point[1])

# def publicar_task_global(pub):
#     if not connected:
#         return
#     payload = {}
#     for tid, task in task_local.items():
#         payload[tid] = {
#             "coleta": (task.coleta_x, task.coleta_y),
#             "destino": (task.dest_x, task.dest_y),
#             "status": task.status,
#             "robot_id": task.robot_id,
#             "dist": distance_to((task.coleta_x, task.coleta_y))
#         }
#     pub.publish(json.dumps(payload))

# def enviar_meta(client, x, y, angulo=0, offset=0.8):
#     dx, dy = 0, offset
#     if robot_pose:
#         angle = robot_pose[2]
#         dx = offset * math.cos(angle)
#         dy = offset * math.sin(angle)
#     goal = MoveBaseGoal()
#     goal.target_pose.header.frame_id = "map"
#     goal.target_pose.header.stamp = rospy.Time.now()
#     goal.target_pose.pose.position.x = x - dx
#     goal.target_pose.pose.position.y = y - dy

#     yaw = math.radians(angulo)
#     quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
#     goal.target_pose.pose.orientation.x = quat[0]
#     goal.target_pose.pose.orientation.y = quat[1]
#     goal.target_pose.pose.orientation.z = quat[2]
#     goal.target_pose.pose.orientation.w = quat[3]

#     client.send_goal(goal)

# def aguardar_chegada(client, timeout=120.0):
#     ok = client.wait_for_result(rospy.Duration(timeout))
#     return ok and client.get_state() == GoalStatus.SUCCEEDED

# def teletransportar(set_pose_proxy, pose, cid, offset_z=0.4):
#     x, y, theta = pose
#     return set_pose_proxy(model_name=cid, x=x, y=y, z=offset_z, a=theta).success

# def container_existe(cid):
#     try:
#         resp = rospy.ServiceProxy("/get_model_pose", GetModelPose)(cid)
#         return resp is not None
#     except rospy.ServiceException:
#         return False

# def registrar_evento(pub, x, y, acao):
#     ev = ColetaEvent()
#     ev.robot_id = robot_name
#     ev.x = x
#     ev.y = y
#     ev.action = acao
#     ev.timestamp = rospy.get_time()
#     pub.publish(ev)

# def publicar_entrega(pub, task):
#     task.status = "entregue"
#     task.timestamp = rospy.get_time()
#     pub.publish(task)

# def voltar_para_origem(client, tolerancia=0.2):
#     if not robot_pose:
#         return
#     dist = math.hypot(robot_pose[0] - origem[0], robot_pose[1] - origem[1])
#     if dist <= tolerancia:
#         rospy.loginfo(f"[{robot_name}] Já está na origem, aguardando tarefas.")
#         return
#     rospy.loginfo(f"[{robot_name}] Nenhuma tarefa. Voltando para origem {origem[:2]}...")
#     enviar_meta(client, origem[0], origem[1], angulo=90, offset=0)
#     if aguardar_chegada(client, timeout=120):
#         rospy.loginfo(f"[{robot_name}] Estacionado no ponto de origem.")
#     else:
#         rospy.logwarn(f"[{robot_name}] Falha ao voltar para origem. Tentando novamente.")
#         rospy.sleep(2.0)
#         voltar_para_origem(client, tolerancia)

# # ==============================
# # LOOP PRINCIPAL
# # ==============================
# if __name__ == "__main__":
#     rospy.init_node("controller_execucao")
#     robot_name = rospy.get_param("~robot_name", "robot_2")
#     origem_param = rospy.get_param("~origem", [0.0, 0.0])
#     origem = (float(origem_param[0]), float(origem_param[1]), 0.0)

#     clear_costmaps = rospy.ServiceProxy(f"/{robot_name}/move_base/clear_costmaps", Empty)
#     rospy.Subscriber(f"/{robot_name}/odom", Odometry, odom_callback)
#     rospy.Subscriber(f"/{robot_name}/task_local", ContainerTask, task_local_callback)
#     rospy.Subscriber(f"{robot_name}/physarum/connections", String, physarum_callback)
#     rospy.Subscriber("/task_global", String, task_global_callback)

#     pub_global = rospy.Publisher("/task_global", String, queue_size=10)
#     pub_coleta = rospy.Publisher("/coleta_events", ColetaEvent, queue_size=10)
#     set_pose_proxy = rospy.ServiceProxy("/set_model_pose", SetModelPose)

#     client = actionlib.SimpleActionClient(f"/{robot_name}/move_base", MoveBaseAction)
#     client.wait_for_server()
#     rate = rospy.Rate(5)

#     while not rospy.is_shutdown():
#         publicar_task_global(pub_global)

#         # Escolher nova tarefa se necessário
#         if not tarefa_atual:
#             for tid, t in task_local.items():
#                 if t.status == "livre":
#                     if not container_existe(t.id):
#                         t.status = "removida"
#                         t.robot_id = ""
#                         publicar_task_global(pub_global)
#                         continue
#                     tarefa_atual = t
#                     tarefa_atual.robot_id = robot_name
#                     tarefa_atual.status = "em_execucao"
#                     publicar_task_global(pub_global)  # Propaga imediatamente
#                     break

#             if not tarefa_atual:
#                 if robot_pose:
#                     dist_origem = math.hypot(robot_pose[0] - origem[0], robot_pose[1] - origem[1])
#                     if dist_origem > 0.2:
#                         voltar_para_origem(client)
#                     else:
#                         rospy.loginfo(f"[{robot_name}] Sem tarefa, aguardando na origem.")
#                 rate.sleep()
#                 continue

#         # Container desapareceu antes de iniciar
#         if not container_existe(tarefa_atual.id):
#             tarefa_atual.status = "removida"
#             tarefa_atual.robot_id = ""
#             publicar_task_global(pub_global)
#             tarefa_atual = None
#             rate.sleep()
#             continue

#         # Vai para coleta
#         # Garantir que a tarefa ainda existe antes de tentar usá-la
#         if not tarefa_atual:
#             rate.sleep()
#             continue

#         ponto_coleta = (tarefa_atual.coleta_x, tarefa_atual.coleta_y)
#         #ponto_coleta = (tarefa_atual.coleta_x, tarefa_atual.coleta_y)
#         registrar_evento(pub_coleta, ponto_coleta[0], ponto_coleta[1], "ocupar")
#         sucesso = False
#         tentativas = 0
#         while not rospy.is_shutdown() and not sucesso and tentativas < 2:
#             enviar_meta(client, *ponto_coleta)
#             sucesso = aguardar_chegada(client, timeout=60)
#             if not sucesso:
#                 tentativas += 1
#                 rospy.logwarn(f"[{robot_name}] Falha ao chegar no ponto {ponto_coleta}, tentativa {tentativas}")
#                 try:
#                     clear_costmaps()
#                 except rospy.ServiceException as e:
#                     rospy.logerr(f"[{robot_name}] Falha ao limpar costmaps: {e}")
#                 rospy.sleep(1.0)

#         if not sucesso:
#             registrar_evento(pub_coleta, ponto_coleta[0], ponto_coleta[1], "liberar")
#             tarefa_atual.status = "livre"
#             tarefa_atual.robot_id = ""
#             publicar_task_global(pub_global)
#             tarefa_atual = None
#             rate.sleep()
#             continue

#         # Verifica container após chegada
#         if not container_existe(tarefa_atual.id):
#             tarefa_atual.status = "removida"
#             tarefa_atual.robot_id = ""
#             publicar_task_global(pub_global)
#             tarefa_atual = None
#             continue

#         # Teletransporta e leva ao destino
#         if robot_pose and teletransportar(set_pose_proxy, robot_pose, tarefa_atual.id):
#             container_acoplado = True
#             tarefa_atual.robot_id = robot_name
#             destino = (tarefa_atual.dest_x, tarefa_atual.dest_y)
#             sucesso = False
#             while not rospy.is_shutdown() and not sucesso:
#                 rospy.logwarn(f"[{robot_name}] Indo para {destino}")
#                 enviar_meta(client, *destino, offset=0)
#                 while not rospy.is_shutdown() and client.get_state() not in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.REJECTED]:
#                     if container_acoplado and robot_pose and tarefa_atual:
#                         teletransportar(set_pose_proxy, robot_pose, tarefa_atual.id)
#                     rate.sleep()
#                 sucesso = (client.get_state() == GoalStatus.SUCCEEDED)
#                 if not sucesso:
#                     rospy.logwarn(f"[{robot_name}] Falha ao chegar ao destino {destino}, tentando novamente...")
#                     try:
#                         clear_costmaps()
#                     except rospy.ServiceException as e:
#                         rospy.logerr(f"[{robot_name}] Falha ao limpar costmaps: {e}")
#                     rospy.sleep(1.0)

#             # Entrega final
#             set_pose_proxy(tarefa_atual.id, -5.8, 1.0, 0.0, 0.0)
#             tarefa_atual.status = "entregue"
#             publicar_task_global(pub_global)

#         registrar_evento(pub_coleta, ponto_coleta[0], ponto_coleta[1], "liberar")
#         tarefa_atual = None
#         container_acoplado = False
#         rate.sleep()

