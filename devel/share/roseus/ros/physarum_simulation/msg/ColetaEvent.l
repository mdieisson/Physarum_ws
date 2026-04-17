;; Auto-generated. Do not edit!


(when (boundp 'physarum_simulation::ColetaEvent)
  (if (not (find-package "PHYSARUM_SIMULATION"))
    (make-package "PHYSARUM_SIMULATION"))
  (shadow 'ColetaEvent (find-package "PHYSARUM_SIMULATION")))
(unless (find-package "PHYSARUM_SIMULATION::COLETAEVENT")
  (make-package "PHYSARUM_SIMULATION::COLETAEVENT"))

(in-package "ROS")
;;//! \htmlinclude ColetaEvent.msg.html


(defclass physarum_simulation::ColetaEvent
  :super ros::object
  :slots (_robot_id _x _y _action _timestamp ))

(defmethod physarum_simulation::ColetaEvent
  (:init
   (&key
    ((:robot_id __robot_id) "")
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:action __action) "")
    ((:timestamp __timestamp) 0.0)
    )
   (send-super :init)
   (setq _robot_id (string __robot_id))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _action (string __action))
   (setq _timestamp (float __timestamp))
   self)
  (:robot_id
   (&optional __robot_id)
   (if __robot_id (setq _robot_id __robot_id)) _robot_id)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:action
   (&optional __action)
   (if __action (setq _action __action)) _action)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:serialization-length
   ()
   (+
    ;; string _robot_id
    4 (length _robot_id)
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; string _action
    4 (length _action)
    ;; float64 _timestamp
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _robot_id
       (write-long (length _robot_id) s) (princ _robot_id s)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _action
       (write-long (length _action) s) (princ _action s)
     ;; float64 _timestamp
       (sys::poke _timestamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _robot_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _robot_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _action
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _action (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _timestamp
     (setq _timestamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get physarum_simulation::ColetaEvent :md5sum-) "c4bf146fc488d7abc41d9dcebee68705")
(setf (get physarum_simulation::ColetaEvent :datatype-) "physarum_simulation/ColetaEvent")
(setf (get physarum_simulation::ColetaEvent :definition-)
      "string robot_id
float32 x
float32 y
string action      # \"ocupar\" ou \"liberar\"
float64 timestamp  # hora do evento
")



(provide :physarum_simulation/ColetaEvent "c4bf146fc488d7abc41d9dcebee68705")


