;; Auto-generated. Do not edit!


(when (boundp 'stage_ros::GetModelPose)
  (if (not (find-package "STAGE_ROS"))
    (make-package "STAGE_ROS"))
  (shadow 'GetModelPose (find-package "STAGE_ROS")))
(unless (find-package "STAGE_ROS::GETMODELPOSE")
  (make-package "STAGE_ROS::GETMODELPOSE"))
(unless (find-package "STAGE_ROS::GETMODELPOSEREQUEST")
  (make-package "STAGE_ROS::GETMODELPOSEREQUEST"))
(unless (find-package "STAGE_ROS::GETMODELPOSERESPONSE")
  (make-package "STAGE_ROS::GETMODELPOSERESPONSE"))

(in-package "ROS")





(defclass stage_ros::GetModelPoseRequest
  :super ros::object
  :slots (_model_name ))

(defmethod stage_ros::GetModelPoseRequest
  (:init
   (&key
    ((:model_name __model_name) "")
    )
   (send-super :init)
   (setq _model_name (string __model_name))
   self)
  (:model_name
   (&optional __model_name)
   (if __model_name (setq _model_name __model_name)) _model_name)
  (:serialization-length
   ()
   (+
    ;; string _model_name
    4 (length _model_name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _model_name
       (write-long (length _model_name) s) (princ _model_name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _model_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _model_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass stage_ros::GetModelPoseResponse
  :super ros::object
  :slots (_x _y _z _a _success ))

(defmethod stage_ros::GetModelPoseResponse
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:a __a) 0.0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _a (float __a))
   (setq _success __success)
   self)
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
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
    4
    ;; float32 _a
    4
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _a
       (sys::poke _a (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _a
     (setq _a (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass stage_ros::GetModelPose
  :super ros::object
  :slots ())

(setf (get stage_ros::GetModelPose :md5sum-) "19acca60f2f0ae24d400ff17db3e5408")
(setf (get stage_ros::GetModelPose :datatype-) "stage_ros/GetModelPose")
(setf (get stage_ros::GetModelPose :request) stage_ros::GetModelPoseRequest)
(setf (get stage_ros::GetModelPose :response) stage_ros::GetModelPoseResponse)

(defmethod stage_ros::GetModelPoseRequest
  (:response () (instance stage_ros::GetModelPoseResponse :init)))

(setf (get stage_ros::GetModelPoseRequest :md5sum-) "19acca60f2f0ae24d400ff17db3e5408")
(setf (get stage_ros::GetModelPoseRequest :datatype-) "stage_ros/GetModelPoseRequest")
(setf (get stage_ros::GetModelPoseRequest :definition-)
      "string model_name
---
float32 x
float32 y
float32 z
float32 a
bool success
")

(setf (get stage_ros::GetModelPoseResponse :md5sum-) "19acca60f2f0ae24d400ff17db3e5408")
(setf (get stage_ros::GetModelPoseResponse :datatype-) "stage_ros/GetModelPoseResponse")
(setf (get stage_ros::GetModelPoseResponse :definition-)
      "string model_name
---
float32 x
float32 y
float32 z
float32 a
bool success
")



(provide :stage_ros/GetModelPose "19acca60f2f0ae24d400ff17db3e5408")


