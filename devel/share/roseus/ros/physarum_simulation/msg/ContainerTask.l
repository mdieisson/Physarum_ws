;; Auto-generated. Do not edit!


(when (boundp 'physarum_simulation::ContainerTask)
  (if (not (find-package "PHYSARUM_SIMULATION"))
    (make-package "PHYSARUM_SIMULATION"))
  (shadow 'ContainerTask (find-package "PHYSARUM_SIMULATION")))
(unless (find-package "PHYSARUM_SIMULATION::CONTAINERTASK")
  (make-package "PHYSARUM_SIMULATION::CONTAINERTASK"))

(in-package "ROS")
;;//! \htmlinclude ContainerTask.msg.html


(defclass physarum_simulation::ContainerTask
  :super ros::object
  :slots (_id _x _y _coleta_x _coleta_y _dest_x _dest_y _status _robot_id _timestamp ))

(defmethod physarum_simulation::ContainerTask
  (:init
   (&key
    ((:id __id) "")
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:coleta_x __coleta_x) 0.0)
    ((:coleta_y __coleta_y) 0.0)
    ((:dest_x __dest_x) 0.0)
    ((:dest_y __dest_y) 0.0)
    ((:status __status) "")
    ((:robot_id __robot_id) "")
    ((:timestamp __timestamp) 0.0)
    )
   (send-super :init)
   (setq _id (string __id))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _coleta_x (float __coleta_x))
   (setq _coleta_y (float __coleta_y))
   (setq _dest_x (float __dest_x))
   (setq _dest_y (float __dest_y))
   (setq _status (string __status))
   (setq _robot_id (string __robot_id))
   (setq _timestamp (float __timestamp))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:coleta_x
   (&optional __coleta_x)
   (if __coleta_x (setq _coleta_x __coleta_x)) _coleta_x)
  (:coleta_y
   (&optional __coleta_y)
   (if __coleta_y (setq _coleta_y __coleta_y)) _coleta_y)
  (:dest_x
   (&optional __dest_x)
   (if __dest_x (setq _dest_x __dest_x)) _dest_x)
  (:dest_y
   (&optional __dest_y)
   (if __dest_y (setq _dest_y __dest_y)) _dest_y)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:robot_id
   (&optional __robot_id)
   (if __robot_id (setq _robot_id __robot_id)) _robot_id)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:serialization-length
   ()
   (+
    ;; string _id
    4 (length _id)
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _coleta_x
    4
    ;; float32 _coleta_y
    4
    ;; float32 _dest_x
    4
    ;; float32 _dest_y
    4
    ;; string _status
    4 (length _status)
    ;; string _robot_id
    4 (length _robot_id)
    ;; float64 _timestamp
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _id
       (write-long (length _id) s) (princ _id s)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _coleta_x
       (sys::poke _coleta_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _coleta_y
       (sys::poke _coleta_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _dest_x
       (sys::poke _dest_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _dest_y
       (sys::poke _dest_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _status
       (write-long (length _status) s) (princ _status s)
     ;; string _robot_id
       (write-long (length _robot_id) s) (princ _robot_id s)
     ;; float64 _timestamp
       (sys::poke _timestamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _coleta_x
     (setq _coleta_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _coleta_y
     (setq _coleta_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _dest_x
     (setq _dest_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _dest_y
     (setq _dest_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _robot_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _robot_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _timestamp
     (setq _timestamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get physarum_simulation::ContainerTask :md5sum-) "4ac7e3eef6a945017db3fb1512eda31d")
(setf (get physarum_simulation::ContainerTask :datatype-) "physarum_simulation/ContainerTask")
(setf (get physarum_simulation::ContainerTask :definition-)
      "string id
float32 x
float32 y
float32 coleta_x     # novo
float32 coleta_y     # novo
float32 dest_x
float32 dest_y
string status
string robot_id
float64 timestamp
")



(provide :physarum_simulation/ContainerTask "4ac7e3eef6a945017db3fb1512eda31d")


