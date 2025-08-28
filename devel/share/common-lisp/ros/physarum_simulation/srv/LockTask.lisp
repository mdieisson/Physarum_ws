; Auto-generated. Do not edit!


(cl:in-package physarum_simulation-srv)


;//! \htmlinclude LockTask-request.msg.html

(cl:defclass <LockTask-request> (roslisp-msg-protocol:ros-message)
  ((task_id
    :reader task_id
    :initarg :task_id
    :type cl:string
    :initform "")
   (robot_id
    :reader robot_id
    :initarg :robot_id
    :type cl:string
    :initform ""))
)

(cl:defclass LockTask-request (<LockTask-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LockTask-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LockTask-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name physarum_simulation-srv:<LockTask-request> is deprecated: use physarum_simulation-srv:LockTask-request instead.")))

(cl:ensure-generic-function 'task_id-val :lambda-list '(m))
(cl:defmethod task_id-val ((m <LockTask-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader physarum_simulation-srv:task_id-val is deprecated.  Use physarum_simulation-srv:task_id instead.")
  (task_id m))

(cl:ensure-generic-function 'robot_id-val :lambda-list '(m))
(cl:defmethod robot_id-val ((m <LockTask-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader physarum_simulation-srv:robot_id-val is deprecated.  Use physarum_simulation-srv:robot_id instead.")
  (robot_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LockTask-request>) ostream)
  "Serializes a message object of type '<LockTask-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'task_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'task_id))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'robot_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'robot_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LockTask-request>) istream)
  "Deserializes a message object of type '<LockTask-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'task_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'task_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'robot_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LockTask-request>)))
  "Returns string type for a service object of type '<LockTask-request>"
  "physarum_simulation/LockTaskRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LockTask-request)))
  "Returns string type for a service object of type 'LockTask-request"
  "physarum_simulation/LockTaskRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LockTask-request>)))
  "Returns md5sum for a message object of type '<LockTask-request>"
  "533c5e19719d65457567739c7f3d4d58")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LockTask-request)))
  "Returns md5sum for a message object of type 'LockTask-request"
  "533c5e19719d65457567739c7f3d4d58")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LockTask-request>)))
  "Returns full string definition for message of type '<LockTask-request>"
  (cl:format cl:nil "string task_id~%string robot_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LockTask-request)))
  "Returns full string definition for message of type 'LockTask-request"
  (cl:format cl:nil "string task_id~%string robot_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LockTask-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'task_id))
     4 (cl:length (cl:slot-value msg 'robot_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LockTask-request>))
  "Converts a ROS message object to a list"
  (cl:list 'LockTask-request
    (cl:cons ':task_id (task_id msg))
    (cl:cons ':robot_id (robot_id msg))
))
;//! \htmlinclude LockTask-response.msg.html

(cl:defclass <LockTask-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass LockTask-response (<LockTask-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LockTask-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LockTask-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name physarum_simulation-srv:<LockTask-response> is deprecated: use physarum_simulation-srv:LockTask-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <LockTask-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader physarum_simulation-srv:success-val is deprecated.  Use physarum_simulation-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LockTask-response>) ostream)
  "Serializes a message object of type '<LockTask-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LockTask-response>) istream)
  "Deserializes a message object of type '<LockTask-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LockTask-response>)))
  "Returns string type for a service object of type '<LockTask-response>"
  "physarum_simulation/LockTaskResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LockTask-response)))
  "Returns string type for a service object of type 'LockTask-response"
  "physarum_simulation/LockTaskResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LockTask-response>)))
  "Returns md5sum for a message object of type '<LockTask-response>"
  "533c5e19719d65457567739c7f3d4d58")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LockTask-response)))
  "Returns md5sum for a message object of type 'LockTask-response"
  "533c5e19719d65457567739c7f3d4d58")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LockTask-response>)))
  "Returns full string definition for message of type '<LockTask-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LockTask-response)))
  "Returns full string definition for message of type 'LockTask-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LockTask-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LockTask-response>))
  "Converts a ROS message object to a list"
  (cl:list 'LockTask-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'LockTask)))
  'LockTask-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'LockTask)))
  'LockTask-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LockTask)))
  "Returns string type for a service object of type '<LockTask>"
  "physarum_simulation/LockTask")