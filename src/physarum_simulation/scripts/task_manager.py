#!/usr/bin/env python3
import rospy, time, math
from physarum_simulation.msg import ContainerTask
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from stage_ros.srv import GetModelPose

# ESTADOS
robot_pose = (0,0,0)
task_list = {}
physarum_neighbors = set()
robot_name = None

# CALLBACKS
def task_cb(msg):
    if (msg.id not in task_list) or (msg.timestamp >= task_list[msg.id].timestamp):
        task_list[msg.id] = msg
        rospy.loginfo(f"[{robot_name}] Task {msg.id} -> {msg.status} por {msg.robot_id}")

def physarum_cb(msg):
    physarum_neighbors.clear()
    for conn in msg.data.split(','):
        if f"{robot_name}<->" in conn or f"<->{robot_name}" in conn:
            parts=conn.split('<->')
            physarum_neighbors.add(parts[0] if parts[1]==robot_name else parts[1])

def odom_cb(msg):
    global robot_pose
    x=msg.pose.pose.position.x; y=msg.pose.pose.position.y
    q=msg.pose.pose.orientation
    siny=2*(q.w*q.z+q.x*q.y); cosy=1-2*(q.y**2+q.z**2)
    robot_pose=(x,y,math.atan2(siny,cosy))

# FUNÇÕES
def euclidean(a,b): return math.hypot(a[0]-b[0],a[1]-b[1])

def publicar_task(task):
    task.timestamp=rospy.get_time()
    pub.publish(task)

def escolher_task():
    livres=[t for t in task_list.values() if t.status=="livre"]
    if not livres: return None
    livres.sort(key=lambda t:(euclidean(robot_pose[:2],(t.x,t.y)),t.id))
    return livres[0]

def verificar_container(cid):
    try:
        rospy.wait_for_service("/get_model_pose",timeout=1)
        resp=rospy.ServiceProxy("/get_model_pose",GetModelPose)(cid)
        return resp is not None
    except rospy.ServiceException: return False

def reservar_task(task):
    if not verificar_container(task.id): return False
    if task.status!="livre": return False
    task.status="em_execucao"; task.robot_id=robot_name; publicar_task(task)
    rospy.sleep(0.5)
    atual=task_list.get(task.id)
    return atual and atual.robot_id==robot_name and atual.status=="em_execucao"

# LOOP PRINCIPAL
if __name__=="__main__":
    rospy.init_node("task_manager")
    robot_name=rospy.get_param("~robot_name","robot_2")
    rospy.Subscriber("/task_updates",ContainerTask,task_cb)
    rospy.Subscriber("/physarum/connections",String,physarum_cb)
    rospy.Subscriber(f"/{robot_name}/odom",Odometry,odom_cb)
    pub=rospy.Publisher("/task_updates",ContainerTask,queue_size=10)
    time.sleep(2)
    rate=rospy.Rate(1)
    while not rospy.is_shutdown():
        executando=[t for t in task_list.values() if t.status=="em_execucao" and t.robot_id==robot_name]
        if not executando:
            t=escolher_task()
            if t and reservar_task(t): rospy.loginfo(f"[{robot_name}] Iniciando task {t.id}")
        if physarum_neighbors:
            for t in task_list.values(): publicar_task(t)
        rate.sleep()
