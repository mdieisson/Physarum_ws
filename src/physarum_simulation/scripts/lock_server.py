#!/usr/bin/env python3
import rospy
from physarum_simulation.srv import LockTask, LockTaskResponse

locked_tasks = {}  # task_id -> robot_id

def handle_lock(req):
    if req.task_id not in locked_tasks:
        locked_tasks[req.task_id] = req.robot_id
        rospy.loginfo(f"Task {req.task_id} locked by {req.robot_id}")
        return LockTaskResponse(success=True)
    else:
        return LockTaskResponse(success=False)

if __name__ == "__main__":
    rospy.init_node("lock_server")
    rospy.Service("/lock_task", LockTask, handle_lock)
    rospy.loginfo("Lock server pronto.")
    rospy.spin()
