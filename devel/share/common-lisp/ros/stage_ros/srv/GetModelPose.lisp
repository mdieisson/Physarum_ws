; Auto-generated. Do not edit!


(cl:in-package stage_ros-srv)


;//! \htmlinclude GetModelPose-request.msg.html

(cl:defclass <GetModelPose-request> (roslisp-msg-protocol:ros-message)
  ((model_name
    :reader model_name
    :initarg :model_name
    :type cl:string
    :initform ""))
)

(cl:defclass GetModelPose-request (<GetModelPose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetModelPose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetModelPose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name stage_ros-srv:<GetModelPose-request> is deprecated: use stage_ros-srv:GetModelPose-request instead.")))

(cl:ensure-generic-function 'model_name-val :lambda-list '(m))
(cl:defmethod model_name-val ((m <GetModelPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stage_ros-srv:model_name-val is deprecated.  Use stage_ros-srv:model_name instead.")
  (model_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetModelPose-request>) ostream)
  "Serializes a message object of type '<GetModelPose-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'model_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'model_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetModelPose-request>) istream)
  "Deserializes a message object of type '<GetModelPose-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'model_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'model_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetModelPose-request>)))
  "Returns string type for a service object of type '<GetModelPose-request>"
  "stage_ros/GetModelPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetModelPose-request)))
  "Returns string type for a service object of type 'GetModelPose-request"
  "stage_ros/GetModelPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetModelPose-request>)))
  "Returns md5sum for a message object of type '<GetModelPose-request>"
  "19acca60f2f0ae24d400ff17db3e5408")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetModelPose-request)))
  "Returns md5sum for a message object of type 'GetModelPose-request"
  "19acca60f2f0ae24d400ff17db3e5408")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetModelPose-request>)))
  "Returns full string definition for message of type '<GetModelPose-request>"
  (cl:format cl:nil "string model_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetModelPose-request)))
  "Returns full string definition for message of type 'GetModelPose-request"
  (cl:format cl:nil "string model_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetModelPose-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'model_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetModelPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetModelPose-request
    (cl:cons ':model_name (model_name msg))
))
;//! \htmlinclude GetModelPose-response.msg.html

(cl:defclass <GetModelPose-response> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (a
    :reader a
    :initarg :a
    :type cl:float
    :initform 0.0)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetModelPose-response (<GetModelPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetModelPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetModelPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name stage_ros-srv:<GetModelPose-response> is deprecated: use stage_ros-srv:GetModelPose-response instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <GetModelPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stage_ros-srv:x-val is deprecated.  Use stage_ros-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <GetModelPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stage_ros-srv:y-val is deprecated.  Use stage_ros-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <GetModelPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stage_ros-srv:z-val is deprecated.  Use stage_ros-srv:z instead.")
  (z m))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <GetModelPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stage_ros-srv:a-val is deprecated.  Use stage_ros-srv:a instead.")
  (a m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetModelPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stage_ros-srv:success-val is deprecated.  Use stage_ros-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetModelPose-response>) ostream)
  "Serializes a message object of type '<GetModelPose-response>"
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'a))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetModelPose-response>) istream)
  "Deserializes a message object of type '<GetModelPose-response>"
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetModelPose-response>)))
  "Returns string type for a service object of type '<GetModelPose-response>"
  "stage_ros/GetModelPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetModelPose-response)))
  "Returns string type for a service object of type 'GetModelPose-response"
  "stage_ros/GetModelPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetModelPose-response>)))
  "Returns md5sum for a message object of type '<GetModelPose-response>"
  "19acca60f2f0ae24d400ff17db3e5408")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetModelPose-response)))
  "Returns md5sum for a message object of type 'GetModelPose-response"
  "19acca60f2f0ae24d400ff17db3e5408")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetModelPose-response>)))
  "Returns full string definition for message of type '<GetModelPose-response>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%float32 a~%bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetModelPose-response)))
  "Returns full string definition for message of type 'GetModelPose-response"
  (cl:format cl:nil "float32 x~%float32 y~%float32 z~%float32 a~%bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetModelPose-response>))
  (cl:+ 0
     4
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetModelPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetModelPose-response
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':a (a msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetModelPose)))
  'GetModelPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetModelPose)))
  'GetModelPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetModelPose)))
  "Returns string type for a service object of type '<GetModelPose>"
  "stage_ros/GetModelPose")