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
import threading
task_lock = threading.Lock()

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
conexoes_ativas = {}

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
        #print(f"[{robot_name}] Conectado a {connected_robot}")
        #physarum_neighbors.add(connected_robot)
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
    ts_novo = data.get("timestamp", 0.0)
    versao = (tid, ts_novo)

    # Ignorar se já processamos exatamente essa versão

    if versao in processed_tasks:
        return
    processed_tasks.add(versao)

    # Criar ou atualizar tarefa
    if tid not in task_local:
        t = ContainerTask()
        t.id = tid
        t.x = data["x"]
        t.y = data["y"]
        t.coleta_x, t.coleta_y = data["coleta"]
        t.dest_x, t.dest_y = data["destino"]
        t.status = data["status"]
        t.robot_id = data.get("robot_id", "")
        t.timestamp = ts_novo
        task_local[tid] = t
    else:
        t = task_local[tid]
        # Atualizar apenas se a versão recebida for mais recente
        if ts_novo > getattr(t, "timestamp", 0.0):
            t.status = data["status"]
            t.robot_id = data.get("robot_id", "")
            t.timestamp = ts_novo
            # Se o robô local estava executando essa tarefa e outro pegou, liberar
            if tarefa_atual and tarefa_atual.id == tid and t.robot_id != robot_name:
                tarefa_atual = None

    # Atualizar visão global também
    task_global[tid] = task_local[tid]

    # Propagar atualização para outros vizinhos conectados
    for neighbor, pub in active_links.items():
        if neighbor != origin:
            pub.publish(msg)
# def task_sync_callback(msg, origin):
#     global processed_tasks, task_global, task_local, tarefa_atual
#     try:
#         data = json.loads(msg.data)
#     except Exception:
#         return
#     tid = str(data.get("id", ""))
#     ts_novo = data.get("timestamp", 0.0)
#     versao = (tid, ts_novo)
#     if versao in processed_tasks:
#         return
#     processed_tasks.add(versao)
#     if tid not in task_local:
#         t = ContainerTask()
#         t.id = tid
#         t.x = data["x"]
#         t.y = data["y"]
#         t.coleta_x, t.coleta_y = data["coleta"]
#         t.dest_x, t.dest_y = data["destino"]
#         t.status = data["status"]
#         t.robot_id = data.get("robot_id", "")
#         t.timestamp = ts_novo
#         task_local[tid] = t
#     else:
#         t = task_local[tid]
#         if ts_novo > getattr(t, "timestamp", 0.0):
#             t.status = data["status"]
#             t.robot_id = data.get("robot_id", "")
#             t.timestamp = ts_novo
#             if tarefa_atual and tarefa_atual.id == tid:
#                 tarefa_atual = None
#     task_global[tid] = task_local[tid]
#     for neighbor, pub in active_links.items():
#         if neighbor != origin:
#             pub.publish(msg)

# ==============================
# FUNÇÕES AUXILIARES
# ==============================
def update_links():
    global active_links, connected_robot, conexoes_ativas
    agora = rospy.get_time()
    
    # Remove links que não existem mais
    for r in list(active_links):
        if r != connected_robot:
            rospy.loginfo(f"[{robot_name}] Desconectado de {r}")
            active_links[r].unregister()
            del active_links[r]
            conexoes_ativas.pop(r, None)
    
    # Cria link novo
    if connected_robot and connected_robot not in active_links:
        topic = f"/link_{'_'.join(sorted([robot_name, connected_robot]))}/task_sync"
        pub = rospy.Publisher(topic, String, queue_size=10)
        active_links[connected_robot] = pub
        conexoes_ativas[connected_robot] = agora
        rospy.Subscriber(topic, String, task_sync_callback, callback_args=connected_robot)
        rospy.loginfo(f"[{robot_name}] Conectado a {connected_robot} no tópico {topic}")
        publish_all_tasks()


def registrar_compartilhamento(tarefa_id, acao, destino=None):
    """
    Registra eventos de compartilhamento de tarefas (envio ou recepção).
    """
    evento = {
        "timestamp": rospy.get_time(),
        "robot": robot_name,
        "tarefa": tarefa_id,
        "acao": acao,  # 'enviado' ou 'recebido'
        "destino": destino
    }
    log_pub.publish(json.dumps(evento))

def publicar_resumo():
    resumo = {
        "robot": robot_name,
        "conexoes": list(conexoes_ativas.keys()),
        "tarefas_locais": [t.id for t in task_local.values()],
        "tarefa_atual": tarefa_atual.id if tarefa_atual else None
    }
    resumo_pub.publish(json.dumps(resumo))
# def update_links():
#     global active_links, connected_robot
#     for r in list(active_links):
#         if r != connected_robot:
#             active_links[r].unregister()
#             del active_links[r]
#     if connected_robot and connected_robot not in active_links:
#         topic = f"/link_{'_'.join(sorted([robot_name, connected_robot]))}/task_sync"
#         pub = rospy.Publisher(topic, String, queue_size=10)
#         active_links[connected_robot] = pub
#         rospy.Subscriber(topic, String, task_sync_callback, callback_args=connected_robot)
#         publish_all_tasks()

def publish_all_tasks():
    with task_lock:
        for t in list(task_local.values()):
            if True:
                payload = {
                    "id": t.id,
                    "x": t.x,
                    "y": t.y,
                    "coleta": (t.coleta_x, t.coleta_y),
                    "destino": (t.dest_x, t.dest_y),
                    "status": t.status,
                    "robot_id": t.robot_id,
                    "timestamp": t.timestamp
                }
                msg = String()
                msg.data = json.dumps(payload)
                tid = t.id
                ts_novo = t.timestamp
                versao = (tid, ts_novo)

                # Ignorar se já processamos exatamente essa versão
                if versao not in processed_tasks:                
                    processed_tasks.add(versao)
                #processed_tasks.add((t.id, t.timestamp))
                for pub in active_links.values():
                    pub.publish(msg)

def publish_task(task: ContainerTask):
    with task_lock:
        payload = {
            "id": task.id,
            "x": task.x,
            "y": task.y,
            "coleta": (task.coleta_x, task.coleta_y),
            "destino": (task.dest_x, task.dest_y),
            "status": task.status,
            "robot_id": task.robot_id,
            "timestamp": task.timestamp
        }
        msg = String()
        msg.data = json.dumps(payload)
        tid = task.id
        ts_novo = task.timestamp
        versao = (tid, ts_novo)

                # Ignorar se já processamos exatamente essa versão
        if versao not in processed_tasks:                
            processed_tasks.add(versao)
        #processed_tasks.add((task.id, task.timestamp))
        for pub in active_links.values():
            pub.publish(msg)

# ==============================
# FUNÇÃO DE NAVEGAÇÃO AJUSTADA
# ==============================
def enviar_meta(client, x, y, angulo=None, offset=0.8):
    """
    Envia um objetivo de navegação ao move_base, priorizando a posição e ignorando a orientação.
    O robô irá parar próximo ao ponto (x,y) sem se alinhar a um ângulo específico.
    """
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    
    # Se uma orientação for fornecida, a utilize.
    # Caso contrário, defina uma orientação neutra (ou a que o robô já está).
    # O move_base pode ignorar a orientação se a tolerância de yaw for grande o suficiente.
    if angulo is not None:
        quat = tf.transformations.quaternion_from_euler(0, 0, math.radians(angulo))
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]
    else:
        # Usa a orientação atual do robô como alvo, fazendo com que ele não gire.
        if robot_pose:
            q = tf.transformations.quaternion_from_euler(0, 0, robot_pose[2])
            goal.target_pose.pose.orientation.x = q[0]
            goal.target_pose.pose.orientation.y = q[1]
            goal.target_pose.pose.orientation.z = q[2]
            goal.target_pose.pose.orientation.w = q[3]
        else:
             # Se a pose não estiver disponível, define uma orientação padrão.
            goal.target_pose.pose.orientation.w = 1.0

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
        enviar_meta(client, origem[0], origem[1], angulo=90)
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
    log_pub = rospy.Publisher("/network_log", String, queue_size=10)
    resumo_pub = rospy.Publisher(f"/{robot_name}/status_resumo", String, queue_size=10)

    pub_coleta = rospy.Publisher("/coleta_events", ColetaEvent, queue_size=10)
    set_pose_proxy = rospy.ServiceProxy("/set_model_pose", SetModelPose)
    client = actionlib.SimpleActionClient(f"/{robot_name}/move_base", MoveBaseAction)
    client.wait_for_server()

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        
        publicar_resumo()
        if task_local:
            publish_all_tasks()
        
        if not tarefa_atual and task_local:
            # Verificar apenas tarefas realmente livres e não alocadas por outros
            for tid, t in list(task_local.items()):
                # Só pega se está livre e ninguém mais está com ela em execução
                if t.status == "livre" and not any(
                    (x.id == t.id and x.status == "em_execução") for x in task_local.values()
                ) and container_existe(t.id):
                    # Assume tarefa
                    tarefa_atual = t
                    tarefa_atual.status = "em_execução"
                    tarefa_atual.robot_id = robot_name
                    tarefa_atual.timestamp = rospy.get_time()

                    # Propaga imediatamente
                    #publish_all_tasks()
                    publish_task(tarefa_atual)
                    break
            

        if not tarefa_atual:
            voltar_para_origem(client)
            rate.sleep()
            continue

        
        #registrar_evento(pub_coleta, ponto_coleta[0], ponto_coleta[1], "ocupar")
        sucesso = False
        rospy.logwarn(f"[{robot_name}] Executando tarefa {tarefa_atual.id}")
        #publish_task(tarefa_atual)
        ponto_coleta = (tarefa_atual.coleta_x, tarefa_atual.coleta_y)
        while not rospy.is_shutdown() and not sucesso and tarefa_atual:
            publicar_resumo()
            enviar_meta(client, *ponto_coleta)
            sucesso= aguardar_chegada(client, timeout=120) 
            if not sucesso:
                rospy.logwarn(f"[{robot_name}] Falha ao chegar no ponto {ponto_coleta}")
                try:
                    clear_costmaps()
                except rospy.ServiceException as e:
                    rospy.logerr(f"[{robot_name}] Falha ao limpar costmaps: {e}")
                rospy.sleep(1.0)


        if not tarefa_atual:
            continue
        posicao_esperada = (tarefa_atual.x, tarefa_atual.y)
        if not container_existe(tarefa_atual.id,posicao_esperada=posicao_esperada):
            tarefa_atual.status = "entregue"
            tarefa_atual.robot_id = ""
            tarefa_atual.timestamp = rospy.get_time()
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

        #registrar_evento(pub_coleta, ponto_coleta[0], ponto_coleta[1], "liberar")
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
# from nav_msgs.msg import Odometry
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from actionlib_msgs.msg import GoalStatus
# from stage_ros.srv import SetModelPose, GetModelPose
# from physarum_simulation.msg import ContainerTask, ColetaEvent
# from std_msgs.msg import String
# from std_srvs.srv import Empty
# import threading
# task_lock = threading.Lock()

# # ==============================
# # VARIÁVEIS GLOBAIS
# # ==============================
# robot_name = None
# robot_pose = None
# origem = (0.0, 0.0, 0.0)
# tarefa_atual = None
# container_acoplado = False

# task_local = {}        # {id: ContainerTask}
# task_global = {}       # {id: ContainerTask}
# processed_tasks = set()

# connected_robot = None
# active_links = {}
# physarum_neighbors = set()
# conexoes_ativas = {}

# verificar_container_ativo = True  # pode ser desativado via ROS param

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
#     publish_all_tasks()

# def physarum_callback(msg):
#     global connected_robot, physarum_neighbors
#     physarum_neighbors.clear()
#     if msg.data.strip():
#         connected_robot = msg.data.strip()
#         #print(f"[{robot_name}] Conectado a {connected_robot}")
#         #physarum_neighbors.add(connected_robot)
#     else:
#         connected_robot = None
#     update_links()
#     publish_all_tasks()
# def task_sync_callback(msg, origin):
#     global processed_tasks, task_global, task_local, tarefa_atual

#     try:
#         data = json.loads(msg.data)
#     except Exception:
#         return

#     tid = str(data.get("id", ""))
#     ts_novo = data.get("timestamp", 0.0)
#     versao = (tid, ts_novo)

#     # Ignorar se já processamos exatamente essa versão

#     if versao in processed_tasks:
#         return
#     processed_tasks.add(versao)

#     # Criar ou atualizar tarefa
#     if tid not in task_local:
#         t = ContainerTask()
#         t.id = tid
#         t.x = data["x"]
#         t.y = data["y"]
#         t.coleta_x, t.coleta_y = data["coleta"]
#         t.dest_x, t.dest_y = data["destino"]
#         t.status = data["status"]
#         t.robot_id = data.get("robot_id", "")
#         t.timestamp = ts_novo
#         task_local[tid] = t
#     else:
#         t = task_local[tid]
#         # Atualizar apenas se a versão recebida for mais recente
#         if ts_novo > getattr(t, "timestamp", 0.0):
#             t.status = data["status"]
#             t.robot_id = data.get("robot_id", "")
#             t.timestamp = ts_novo
#             # Se o robô local estava executando essa tarefa e outro pegou, liberar
#             if tarefa_atual and tarefa_atual.id == tid and t.robot_id != robot_name:
#                 tarefa_atual = None

#     # Atualizar visão global também
#     task_global[tid] = task_local[tid]

#     # Propagar atualização para outros vizinhos conectados
#     for neighbor, pub in active_links.items():
#         if neighbor != origin:
#             pub.publish(msg)
# # def task_sync_callback(msg, origin):
# #     global processed_tasks, task_global, task_local, tarefa_atual
# #     try:
# #         data = json.loads(msg.data)
# #     except Exception:
# #         return
# #     tid = str(data.get("id", ""))
# #     ts_novo = data.get("timestamp", 0.0)
# #     versao = (tid, ts_novo)
# #     if versao in processed_tasks:
# #         return
# #     processed_tasks.add(versao)
# #     if tid not in task_local:
# #         t = ContainerTask()
# #         t.id = tid
# #         t.x = data["x"]
# #         t.y = data["y"]
# #         t.coleta_x, t.coleta_y = data["coleta"]
# #         t.dest_x, t.dest_y = data["destino"]
# #         t.status = data["status"]
# #         t.robot_id = data.get("robot_id", "")
# #         t.timestamp = ts_novo
# #         task_local[tid] = t
# #     else:
# #         t = task_local[tid]
# #         if ts_novo > getattr(t, "timestamp", 0.0):
# #             t.status = data["status"]
# #             t.robot_id = data.get("robot_id", "")
# #             t.timestamp = ts_novo
# #             if tarefa_atual and tarefa_atual.id == tid:
# #                 tarefa_atual = None
# #     task_global[tid] = task_local[tid]
# #     for neighbor, pub in active_links.items():
# #         if neighbor != origin:
# #             pub.publish(msg)

# # ==============================
# # FUNÇÕES AUXILIARES
# # ==============================
# def update_links():
#     global active_links, connected_robot, conexoes_ativas
#     agora = rospy.get_time()
    
#     # Remove links que não existem mais
#     for r in list(active_links):
#         if r != connected_robot:
#             rospy.loginfo(f"[{robot_name}] Desconectado de {r}")
#             active_links[r].unregister()
#             del active_links[r]
#             conexoes_ativas.pop(r, None)
    
#     # Cria link novo
#     if connected_robot and connected_robot not in active_links:
#         topic = f"/link_{'_'.join(sorted([robot_name, connected_robot]))}/task_sync"
#         pub = rospy.Publisher(topic, String, queue_size=10)
#         active_links[connected_robot] = pub
#         conexoes_ativas[connected_robot] = agora
#         rospy.Subscriber(topic, String, task_sync_callback, callback_args=connected_robot)
#         rospy.loginfo(f"[{robot_name}] Conectado a {connected_robot} no tópico {topic}")
#         publish_all_tasks()


# def registrar_compartilhamento(tarefa_id, acao, destino=None):
#     """
#     Registra eventos de compartilhamento de tarefas (envio ou recepção).
#     """
#     evento = {
#         "timestamp": rospy.get_time(),
#         "robot": robot_name,
#         "tarefa": tarefa_id,
#         "acao": acao,  # 'enviado' ou 'recebido'
#         "destino": destino
#     }
#     log_pub.publish(json.dumps(evento))

# def publicar_resumo():
#     resumo = {
#         "robot": robot_name,
#         "conexoes": list(conexoes_ativas.keys()),
#         "tarefas_locais": [t.id for t in task_local.values()],
#         "tarefa_atual": tarefa_atual.id if tarefa_atual else None
#     }
#     resumo_pub.publish(json.dumps(resumo))
# # def update_links():
# #     global active_links, connected_robot
# #     for r in list(active_links):
# #         if r != connected_robot:
# #             active_links[r].unregister()
# #             del active_links[r]
# #     if connected_robot and connected_robot not in active_links:
# #         topic = f"/link_{'_'.join(sorted([robot_name, connected_robot]))}/task_sync"
# #         pub = rospy.Publisher(topic, String, queue_size=10)
# #         active_links[connected_robot] = pub
# #         rospy.Subscriber(topic, String, task_sync_callback, callback_args=connected_robot)
# #         publish_all_tasks()

# def publish_all_tasks():
#     with task_lock:
#         for t in list(task_local.values()):
#             if True:
#                 payload = {
#                     "id": t.id,
#                     "x": t.x,
#                     "y": t.y,
#                     "coleta": (t.coleta_x, t.coleta_y),
#                     "destino": (t.dest_x, t.dest_y),
#                     "status": t.status,
#                     "robot_id": t.robot_id,
#                     "timestamp": t.timestamp
#                 }
#                 msg = String()
#                 msg.data = json.dumps(payload)
#                 tid = t.id
#                 ts_novo = t.timestamp
#                 versao = (tid, ts_novo)

#                 # Ignorar se já processamos exatamente essa versão
#                 if versao not in processed_tasks:                
#                     processed_tasks.add(versao)
#                 #processed_tasks.add((t.id, t.timestamp))
#                 for pub in active_links.values():
#                     pub.publish(msg)

# def publish_task(task: ContainerTask):
#     with task_lock:
#         payload = {
#             "id": task.id,
#             "x": task.x,
#             "y": task.y,
#             "coleta": (task.coleta_x, task.coleta_y),
#             "destino": (task.dest_x, task.dest_y),
#             "status": task.status,
#             "robot_id": task.robot_id,
#             "timestamp": task.timestamp
#         }
#         msg = String()
#         msg.data = json.dumps(payload)
#         tid = task.id
#         ts_novo = task.timestamp
#         versao = (tid, ts_novo)

#                 # Ignorar se já processamos exatamente essa versão
#         if versao not in processed_tasks:                
#             processed_tasks.add(versao)
#         #processed_tasks.add((task.id, task.timestamp))
#         for pub in active_links.values():
#             pub.publish(msg)

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
#     quat = tf.transformations.quaternion_from_euler(0, 0, math.radians(angulo))
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

# def container_existe(cid, posicao_esperada=None, tolerancia=0.5):
#     """
#     Verifica se o container existe e (opcionalmente) se está próximo da posição esperada.
#     Se verificar_container_ativo for False, retorna True sempre (para testes).
#     """
#     if not verificar_container_ativo:
#         return True
#     try:
#         resp = rospy.ServiceProxy("/get_model_pose", GetModelPose)(cid)
#         if resp is None:
#             return False
#         if posicao_esperada:
#             dx = resp.x - posicao_esperada[0]
#             dy = resp.y - posicao_esperada[1]
#             dist = (dx ** 2 + dy ** 2) ** 0.5
#             return dist <= tolerancia
#         return True
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
#     if not robot_pose:
#         return
#     dist = math.hypot(robot_pose[0] - origem[0], robot_pose[1] - origem[1])
#     if dist > tolerancia:
#         enviar_meta(client, origem[0], origem[1], angulo=90, offset=0)
#         aguardar_chegada(client, timeout=120)

# # ==============================
# # LOOP PRINCIPAL
# # ==============================
# if __name__ == "__main__":
#     rospy.init_node("controller_execucao")
#     robot_name = rospy.get_param("~robot_name", "robot_0")
#     origem_param = rospy.get_param("~origem", [0.0, 0.0])
#     origem = (float(origem_param[0]), float(origem_param[1]), 0.0)
#     verificar_container_ativo = rospy.get_param("~verificar_container", True)

#     clear_costmaps = rospy.ServiceProxy(f"/{robot_name}/move_base/clear_costmaps", Empty)
#     rospy.Subscriber(f"/{robot_name}/odom", Odometry, odom_callback)
#     rospy.Subscriber(f"/{robot_name}/task_local", ContainerTask, task_local_callback)
#     rospy.Subscriber(f"/{robot_name}/physarum/connections", String, physarum_callback)
#     log_pub = rospy.Publisher("/network_log", String, queue_size=10)
#     resumo_pub = rospy.Publisher(f"/{robot_name}/status_resumo", String, queue_size=10)

#     pub_coleta = rospy.Publisher("/coleta_events", ColetaEvent, queue_size=10)
#     set_pose_proxy = rospy.ServiceProxy("/set_model_pose", SetModelPose)
#     client = actionlib.SimpleActionClient(f"/{robot_name}/move_base", MoveBaseAction)
#     client.wait_for_server()

#     rate = rospy.Rate(5)
#     while not rospy.is_shutdown():
        
#         publicar_resumo()
#         if task_local:
#             publish_all_tasks()
        
#         if not tarefa_atual and task_local:
#             # Verificar apenas tarefas realmente livres e não alocadas por outros
#             for tid, t in list(task_local.items()):
#                 # Só pega se está livre e ninguém mais está com ela em execução
#                 if t.status == "livre" and not any(
#                     (x.id == t.id and x.status == "em_execução") for x in task_local.values()
#                 ) and container_existe(t.id):
#                     # Assume tarefa
#                     tarefa_atual = t
#                     tarefa_atual.status = "em_execução"
#                     tarefa_atual.robot_id = robot_name
#                     tarefa_atual.timestamp = rospy.get_time()

#                     # Propaga imediatamente
#                     #publish_all_tasks()
#                     publish_task(tarefa_atual)
#                     break
            

#         if not tarefa_atual:
#             voltar_para_origem(client)
#             rate.sleep()
#             continue

        
#         #registrar_evento(pub_coleta, ponto_coleta[0], ponto_coleta[1], "ocupar")
#         sucesso = False
#         rospy.logwarn(f"[{robot_name}] Executando tarefa {tarefa_atual.id}")
#         #publish_task(tarefa_atual)
#         ponto_coleta = (tarefa_atual.coleta_x, tarefa_atual.coleta_y)
#         while not rospy.is_shutdown() and not sucesso and tarefa_atual:
#             publicar_resumo()
#             enviar_meta(client, *ponto_coleta)
#             sucesso= aguardar_chegada(client, timeout=120) 
#             if not sucesso:
#                 rospy.logwarn(f"[{robot_name}] Falha ao chegar no ponto {ponto_coleta}")
#                 try:
#                     clear_costmaps()
#                 except rospy.ServiceException as e:
#                     rospy.logerr(f"[{robot_name}] Falha ao limpar costmaps: {e}")
#                 rospy.sleep(1.0)


#         if not tarefa_atual:
#             continue
#         posicao_esperada = (tarefa_atual.x, tarefa_atual.y)
#         if not container_existe(tarefa_atual.id,posicao_esperada=posicao_esperada):
#             tarefa_atual.status = "entregue"
#             tarefa_atual.robot_id = ""
#             tarefa_atual.timestamp = rospy.get_time()
#             publish_task(tarefa_atual)
#             tarefa_atual = None
#             continue

#         if teletransportar(set_pose_proxy, robot_pose, tarefa_atual.id):
#             container_acoplado = True
#             destino = (tarefa_atual.dest_x, tarefa_atual.dest_y)
#             enviar_meta(client, *destino, offset=0)
#             while not rospy.is_shutdown() and client.get_state() not in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED]:
#                 if container_acoplado:
#                     teletransportar(set_pose_proxy, robot_pose, tarefa_atual.id)
#                 rate.sleep()
#             sucesso = (client.get_state() == GoalStatus.SUCCEEDED)
#             if not sucesso:
#                 rospy.logwarn(f"[{robot_name}] Falha ao chegar ao destino {destino}, tentando novamente...")
#                 try:
#                     clear_costmaps()
#                 except rospy.ServiceException as e:
#                     rospy.logerr(f"[{robot_name}] Falha ao limpar costmaps: {e}")
#                 rospy.sleep(1.0)
#             set_pose_proxy(tarefa_atual.id, -5.8, 1.0, 0.0, 0.0)
#             tarefa_atual.status = "entregue"
#             publish_task(tarefa_atual)

#         #registrar_evento(pub_coleta, ponto_coleta[0], ponto_coleta[1], "liberar")
#         tarefa_atual = None
#         container_acoplado = False
#         publish_all_tasks()
#         rate.sleep()