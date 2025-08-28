#!/usr/bin/env python3
import rospy
from physarum_simulation.msg import ContainerTask
import time

# Lista de tarefas replicada entre os robôs
task_list = {}

def task_update_callback(msg):
    global task_list
    t_id = msg.id
    if (t_id not in task_list) or (msg.timestamp >= task_list[t_id].timestamp):
        task_list[t_id] = msg
        rospy.loginfo(f"[{robot_name}] Atualizou task {t_id}: {msg.status} (por {msg.robot_id})")

def publicar_task(task):
    task.timestamp = rospy.get_time()
    pub.publish(task)
    rospy.loginfo(f"[{robot_name}] Publicou update: {task.id} -> {task.status}")

def escolher_task_livre():
    livres = [t for t in task_list.values() if t.status == "livre"]
    return sorted(livres, key=lambda t: t.id)[0] if livres else None

def tentar_reservar_task(task):
    if task.status != "livre":
        return False
    task.status = "em_execucao"
    task.robot_id = robot_name
    publicar_task(task)
    rospy.sleep(0.5)
    atual = task_list.get(task.id, None)
    if atual and atual.robot_id == robot_name and atual.status == "em_execucao":
        rospy.loginfo(f"[{robot_name}] Reservou com sucesso a task {task.id}")
        return True
    rospy.logwarn(f"[{robot_name}] Perdeu disputa pela task {task.id}")
    return False

if __name__ == "__main__":
    rospy.init_node("task_manager", anonymous=True)
    robot_name = rospy.get_param("~robot_name", "robot_2")

    rospy.Subscriber("/task_updates", ContainerTask, task_update_callback)
    pub = rospy.Publisher("/task_updates", ContainerTask, queue_size=10)

    rate = rospy.Rate(1)
    rospy.loginfo(f"[{robot_name}] Task manager iniciado.")
    time.sleep(2)

    while not rospy.is_shutdown():
        executando = [t for t in task_list.values() if t.status == "em_execucao" and t.robot_id == robot_name]
        if not executando:
            task = escolher_task_livre()
            if task and tentar_reservar_task(task):
                rospy.loginfo(f"[{robot_name}] Iniciando execução da task {task.id}")
        rate.sleep()