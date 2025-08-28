; Auto-generated. Do not edit!


(cl:in-package physarum_simulation-msg)


;//! \htmlinclude ColetaEvent.msg.html

(cl:defclass <ColetaEvent> (roslisp-msg-protocol:ros-message)
  ((robot_id
    :reader robot_id
    :initarg :robot_id
    :type cl:string
    :initform "")
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (action
    :reader action
    :initarg :action
    :type cl:string
    :initform "")
   (timestamp
    :reader timestamp
    :initarg :timestamp
    :type cl:float
    :initform 0.0))
)

(cl:defclass ColetaEvent (<ColetaEvent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ColetaEvent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ColetaEvent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name physarum_simulation-msg:<ColetaEvent> is deprecated: use physarum_simulation-msg:ColetaEvent instead.")))

(cl:ensure-generic-function 'robot_id-val :lambda-list '(m))
(cl:defmethod robot_id-val ((m <ColetaEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader physarum_simulation-msg:robot_id-val is deprecated.  Use physarum_simulation-msg:robot_id instead.")
  (robot_id m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <ColetaEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader physarum_simulation-msg:x-val is deprecated.  Use physarum_simulation-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <ColetaEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader physarum_simulation-msg:y-val is deprecated.  Use physarum_simulation-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'action-val :lambda-list '(m))
(cl:defmethod action-val ((m <ColetaEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader physarum_simulation-msg:action-val is deprecated.  Use physarum_simulation-msg:action instead.")
  (action m))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <ColetaEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader physarum_simulation-msg:timestamp-val is deprecated.  Use physarum_simulation-msg:timestamp instead.")
  (timestamp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ColetaEvent>) ostream)
  "Serializes a message object of type '<ColetaEvent>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'robot_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'robot_id))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'action))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'action))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'timestamp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ColetaEvent>) istream)
  "Deserializes a message object of type '<ColetaEvent>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'robot_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'action) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'action) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'timestamp) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ColetaEvent>)))
  "Returns string type for a message object of type '<ColetaEvent>"
  "physarum_simulation/ColetaEvent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ColetaEvent)))
  "Returns string type for a message object of type 'ColetaEvent"
  "physarum_simulation/ColetaEvent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ColetaEvent>)))
  "Returns md5sum for a message object of type '<ColetaEvent>"
  "c4bf146fc488d7abc41d9dcebee68705")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ColetaEvent)))
  "Returns md5sum for a message object of type 'ColetaEvent"
  "c4bf146fc488d7abc41d9dcebee68705")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ColetaEvent>)))
  "Returns full string definition for message of type '<ColetaEvent>"
  (cl:format cl:nil "string robot_id~%float32 x~%float32 y~%string action      # \"ocupar\" ou \"liberar\"~%float64 timestamp  # hora do evento~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ColetaEvent)))
  "Returns full string definition for message of type 'ColetaEvent"
  (cl:format cl:nil "string robot_id~%float32 x~%float32 y~%string action      # \"ocupar\" ou \"liberar\"~%float64 timestamp  # hora do evento~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ColetaEvent>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'robot_id))
     4
     4
     4 (cl:length (cl:slot-value msg 'action))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ColetaEvent>))
  "Converts a ROS message object to a list"
  (cl:list 'ColetaEvent
    (cl:cons ':robot_id (robot_id msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':action (action msg))
    (cl:cons ':timestamp (timestamp msg))
))
