;; Auto-generated. Do not edit!


(when (boundp 'stage_ros::SetModelPose)
  (if (not (find-package "STAGE_ROS"))
    (make-package "STAGE_ROS"))
  (shadow 'SetModelPose (find-package "STAGE_ROS")))
(unless (find-package "STAGE_ROS::SETMODELPOSE")
  (make-package "STAGE_ROS::SETMODELPOSE"))
(unless (find-package "STAGE_ROS::SETMODELPOSEREQUEST")
  (make-package "STAGE_ROS::SETMODELPOSEREQUEST"))
(unless (find-package "STAGE_ROS::SETMODELPOSERESPONSE")
  (make-package "STAGE_ROS::SETMODELPOSERESPONSE"))

(in-package "ROS")





(defclass stage_ros::SetModelPoseRequest
  :super ros::object
  :slots (_model_name _x _y _z _a ))

(defmethod stage_ros::SetModelPoseRequest
  (:init
   (&key
    ((:model_name __model_name) "")
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:a __a) 0.0)
    )
   (send-super :init)
   (setq _model_name (string __model_name))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _a (float __a))
   self)
  (:model_name
   (&optional __model_name)
   (if __model_name (setq _model_name __model_name)) _model_name)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:a
   (&optional __a)
   (if __a (setq _a __a)) _a)
  (:serialization-length
   ()
   (+
    ;; string _model_name
    4 (length _model_name)
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
    4
    ;; float32 _a
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _model_name
       (write-long (length _model_name) s) (princ _model_name s)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _a
       (sys::poke _a (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _model_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _model_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _a
     (setq _a (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass stage_ros::SetModelPoseResponse
  :super ros::object
  :slots (_success ))

(defmethod stage_ros::SetModelPoseResponse
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

(defclass stage_ros::SetModelPose
  :super ros::object
  :slots ())

(setf (get stage_ros::SetModelPose :md5sum-) "c397a50cbfa6253f64e0ed985ef4b9c0")
(setf (get stage_ros::SetModelPose :datatype-) "stage_ros/SetModelPose")
(setf (get stage_ros::SetModelPose :request) stage_ros::SetModelPoseRequest)
(setf (get stage_ros::SetModelPose :response) stage_ros::SetModelPoseResponse)

(defmethod stage_ros::SetModelPoseRequest
  (:response () (instance stage_ros::SetModelPoseResponse :init)))

(setf (get stage_ros::SetModelPoseRequest :md5sum-) "c397a50cbfa6253f64e0ed985ef4b9c0")
(setf (get stage_ros::SetModelPoseRequest :datatype-) "stage_ros/SetModelPoseRequest")
(setf (get stage_ros::SetModelPoseRequest :definition-)
      "string model_name
float32 x
float32 y
float32 z
float32 a
---
bool success
")

(setf (get stage_ros::SetModelPoseResponse :md5sum-) "c397a50cbfa6253f64e0ed985ef4b9c0")
(setf (get stage_ros::SetModelPoseResponse :datatype-) "stage_ros/SetModelPoseResponse")
(setf (get stage_ros::SetModelPoseResponse :definition-)
      "string model_name
float32 x
float32 y
float32 z
float32 a
---
bool success
")



(provide :stage_ros/SetModelPose "c397a50cbfa6253f64e0ed985ef4b9c0")


