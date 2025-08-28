#!/usr/bin/env python3
import rospy
import random
import time
from physarum_simulation.msg import ContainerTask
from stage_ros.srv import GetModelPose

# ==========================
# CONFIGURAÇÃO
# ==========================
N = 10  # número de containers a spawnar (ajustado para 15)
container_prefix = "caixa_0_"
robots = ["robot_0","robot_1","robot_2"]  # lista de robôs

# --- MODIFICAÇÃO PRINCIPAL: DEFINIÇÃO DE DISTRIBUIÇÃO FIXA E DETERMINÍSTICA ---
# Dicionário que define quantas tarefas cada robô receberá.
# A ordem de atribuição será sequencial.
distribuicao_tarefas_por_robo = {
    "robot_0": 10,
    "robot_1": 0,
    "robot_2": 0
}
# Verifique se o total de tarefas corresponde a N
if sum(distribuicao_tarefas_por_robo.values()) != N:
    rospy.logerr(f"Erro: A soma das tarefas na distribuicao_tarefas_por_robo ({sum(distribuicao_tarefas_por_robo.values())}) não é igual a N ({N}). Por favor, ajuste a configuração.")
    exit()

# Pontos de coleta (podem se repetir aleatoriamente)
pontos_coleta = [(-4, 4.0), (0, 4.0), (4, 4.0)]

# Posições de estoque (podem se repetir aleatoriamente)
posicoes_estoque = [
    (-5.8, -0.45), (-5.8, 2.20), (-5.8, -3.20), (-5.8, -5.82),
    (1.85, 2.20), (1.85, -0.45), (1.85, -3.20), (1.85, -5.82)
]

# Dicionário de publishers para cada robô
pubs = {}

# ==========================
# FUNÇÕES AUXILIARES
# ==========================
def get_pose(container_name):
    """Obtém a posição atual do container no Stage."""
    try:
        rospy.wait_for_service("/get_model_pose", timeout=3.0)
        resp = rospy.ServiceProxy("/get_model_pose", GetModelPose)(container_name)
        return resp.x, resp.y
    except rospy.ServiceException as e:
        rospy.logerr(f"Erro ao obter pose de {container_name}: {e}")
        return None, None

def criar_task(container_id, x, y):
    """Cria uma ContainerTask com ponto de coleta e destino (ambos sorteados aleatoriamente)."""
    task = ContainerTask()
    task.id = str(container_id)
    task.x = x
    task.y = y

    # Sorteia ponto de coleta (pode repetir)
    coleta_x, coleta_y = random.choice(pontos_coleta)
    task.coleta_x = coleta_x
    task.coleta_y = coleta_y

    # Sorteia ponto de estoque (pode repetir)
    dest_x, dest_y = random.choice(posicoes_estoque)
    task.dest_x = dest_x
    task.dest_y = dest_y

    task.status = "livre"
    task.robot_id = "unassigned"
    task.timestamp = rospy.get_time()
    return task

# ==========================
# LOOP PRINCIPAL
# ==========================
if __name__ == "__main__":
    rospy.init_node("task_spawner", anonymous=True)

    # Cria um publisher para cada robô
    for rob in robots:
        pubs[rob] = rospy.Publisher(f"/{rob}/task_local", ContainerTask, queue_size=10)

    rospy.sleep(1.0)  # Garante que os publishers foram criados

    # Cria a lista de robôs para a distribuição sequencial
    tarefas_para_distribuir = []
    for rob, num_tarefas in distribuicao_tarefas_por_robo.items():
        tarefas_para_distribuir.extend([rob] * num_tarefas)
    
    # NÃO embaralha a lista. A ordem é fixa.
    
    for i in range(N):
        nome = f"{container_prefix}{i}"
        x, y = get_pose(nome)
        if x is not None:
            task = criar_task(nome, x, y)
            
            # --- MODIFICAÇÃO PRINCIPAL: DISTRIBUIÇÃO A PARTIR DA LISTA ORDENADA ---
            rob = tarefas_para_distribuir[i]
            
            pubs[rob].publish(task)
            rospy.loginfo(
                f"[Spawner] Tarefa {task.id} enviada para {rob}: coleta ({task.coleta_x:.2f},{task.coleta_y:.2f}), "
                f"destino ({task.dest_x:.2f},{task.dest_y:.2f})"
            )
        else:
            rospy.logwarn(f"[Spawner] Container {nome} não encontrado")
# #!/usr/bin/env python3
# import rospy
# import random
# import time
# from physarum_simulation.msg import ContainerTask
# from stage_ros.srv import GetModelPose

# # ==========================
# # CONFIGURAÇÃO
# # ==========================
# N = 27  # número de containers a spawnar
# container_prefix = "caixa_0_"
# robots = ["robot_0","robot_1","robot_2"]  # lista de robôs

# # Pontos de coleta (podem se repetir aleatoriamente)
# pontos_coleta = [(-4, 4.0), (0, 4.0), (4, 4.0)]

# # Posições de estoque (podem se repetir aleatoriamente)
# posicoes_estoque = [
#     (-5.8, -0.45), (-5.8, 2.20), (-5.8, -3.20), (-5.8, -5.82),
#     (1.85, 2.20), (1.85, -0.45), (1.85, -3.20), (1.85, -5.82)
# ]

# # Dicionário de publishers para cada robô
# pubs = {}

# # ==========================
# # FUNÇÕES AUXILIARES
# # ==========================
# def get_pose(container_name):
#     """Obtém a posição atual do container no Stage."""
#     try:
#         rospy.wait_for_service("/get_model_pose", timeout=3.0)
#         resp = rospy.ServiceProxy("/get_model_pose", GetModelPose)(container_name)
#         return resp.x, resp.y
#     except rospy.ServiceException as e:
#         rospy.logerr(f"Erro ao obter pose de {container_name}: {e}")
#         return None, None

# def criar_task(container_id, x, y):
#     """Cria uma ContainerTask com ponto de coleta e destino (ambos sorteados aleatoriamente)."""
#     task = ContainerTask()
#     task.id = str(container_id)
#     task.x = x
#     task.y = y

#     # Sorteia ponto de coleta (pode repetir)
#     coleta_x, coleta_y = random.choice(pontos_coleta)
#     task.coleta_x = coleta_x
#     task.coleta_y = coleta_y

#     # Sorteia ponto de estoque (pode repetir)
#     dest_x, dest_y = random.choice(posicoes_estoque)
#     task.dest_x = dest_x
#     task.dest_y = dest_y

#     task.status = "livre"
#     task.robot_id = "unassigned"
#     task.timestamp = rospy.get_time()
#     return task

# # ==========================
# # LOOP PRINCIPAL
# # ==========================
# if __name__ == "__main__":
#     rospy.init_node("task_spawner", anonymous=True)

#     # Cria um publisher para cada robô
#     for rob in robots:
#         pubs[rob] = rospy.Publisher(f"/{rob}/task_local", ContainerTask, queue_size=10)

#     rospy.sleep(1.0)  # garante que os publishers foram criados

#     for i in range(N):
#         nome = f"{container_prefix}{i}"
#         x, y = get_pose(nome)
#         if x is not None:
#             task = criar_task(nome, x, y)
#             # Sorteia um robô para a tarefa
#             rob = random.choice(robots)
#             pubs[rob].publish(task)
#             rospy.loginfo(
#                 f"[Spawner] Tarefa {task.id} enviada para {rob}: coleta ({task.coleta_x:.2f},{task.coleta_y:.2f}), "
#                 f"destino ({task.dest_x:.2f},{task.dest_y:.2f})"
#             )
#         else:
#             rospy.logwarn(f"[Spawner] Container {nome} não encontrado")
