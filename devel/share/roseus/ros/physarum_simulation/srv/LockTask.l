;; Auto-generated. Do not edit!


(when (boundp 'physarum_simulation::LockTask)
  (if (not (find-package "PHYSARUM_SIMULATION"))
    (make-package "PHYSARUM_SIMULATION"))
  (shadow 'LockTask (find-package "PHYSARUM_SIMULATION")))
(unless (find-package "PHYSARUM_SIMULATION::LOCKTASK")
  (make-package "PHYSARUM_SIMULATION::LOCKTASK"))
(unless (find-package "PHYSARUM_SIMULATION::LOCKTASKREQUEST")
  (make-package "PHYSARUM_SIMULATION::LOCKTASKREQUEST"))
(unless (find-package "PHYSARUM_SIMULATION::LOCKTASKRESPONSE")
  (make-package "PHYSARUM_SIMULATION::LOCKTASKRESPONSE"))

(in-package "ROS")





(defclass physarum_simulation::LockTaskRequest
  :super ros::object
  :slots (_task_id _robot_id ))

(defmethod physarum_simulation::LockTaskRequest
  (:init
   (&key
    ((:task_id __task_id) "")
    ((:robot_id __robot_id) "")
    )
   (send-super :init)
   (setq _task_id (string __task_id))
   (setq _robot_id (string __robot_id))
   self)
  (:task_id
   (&optional __task_id)
   (if __task_id (setq _task_id __task_id)) _task_id)
  (:robot_id
   (&optional __robot_id)
   (if __robot_id (setq _robot_id __robot_id)) _robot_id)
  (:serialization-length
   ()
   (+
    ;; string _task_id
    4 (length _task_id)
    ;; string _robot_id
    4 (length _robot_id)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _task_id
       (write-long (length _task_id) s) (princ _task_id s)
     ;; string _robot_id
       (write-long (length _robot_id) s) (princ _robot_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _task_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _task_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _robot_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _robot_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass physarum_simulation::LockTaskResponse
  :super ros::object
  :slots (_success ))

(defmethod physarum_simulation::LockTaskResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass physarum_simulation::LockTask
  :super ros::object
  :slots ())

(setf (get physarum_simulation::LockTask :md5sum-) "533c5e19719d65457567739c7f3d4d58")
(setf (get physarum_simulation::LockTask :datatype-) "physarum_simulation/LockTask")
(setf (get physarum_simulation::LockTask :request) physarum_simulation::LockTaskRequest)
(setf (get physarum_simulation::LockTask :response) physarum_simulation::LockTaskResponse)

(defmethod physarum_simulation::LockTaskRequest
  (:response () (instance physarum_simulation::LockTaskResponse :init)))

(setf (get physarum_simulation::LockTaskRequest :md5sum-) "533c5e19719d65457567739c7f3d4d58")
(setf (get physarum_simulation::LockTaskRequest :datatype-) "physarum_simulation/LockTaskRequest")
(setf (get physarum_simulation::LockTaskRequest :definition-)
      "string task_id
string robot_id
---
bool success

")

(setf (get physarum_simulation::LockTaskResponse :md5sum-) "533c5e19719d65457567739c7f3d4d58")
(setf (get physarum_simulation::LockTaskResponse :datatype-) "physarum_simulation/LockTaskResponse")
(setf (get physarum_simulation::LockTaskResponse :definition-)
      "string task_id
string robot_id
---
bool success

")



(provide :physarum_simulation/LockTask "533c5e19719d65457567739c7f3d4d58")


