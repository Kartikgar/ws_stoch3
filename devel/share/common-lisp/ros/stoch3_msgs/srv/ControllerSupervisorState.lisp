; Auto-generated. Do not edit!


(cl:in-package stoch3_msgs-srv)


;//! \htmlinclude ControllerSupervisorState-request.msg.html

(cl:defclass <ControllerSupervisorState-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ControllerSupervisorState-request (<ControllerSupervisorState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerSupervisorState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerSupervisorState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name stoch3_msgs-srv:<ControllerSupervisorState-request> is deprecated: use stoch3_msgs-srv:ControllerSupervisorState-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerSupervisorState-request>) ostream)
  "Serializes a message object of type '<ControllerSupervisorState-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerSupervisorState-request>) istream)
  "Deserializes a message object of type '<ControllerSupervisorState-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerSupervisorState-request>)))
  "Returns string type for a service object of type '<ControllerSupervisorState-request>"
  "stoch3_msgs/ControllerSupervisorStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerSupervisorState-request)))
  "Returns string type for a service object of type 'ControllerSupervisorState-request"
  "stoch3_msgs/ControllerSupervisorStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerSupervisorState-request>)))
  "Returns md5sum for a message object of type '<ControllerSupervisorState-request>"
  "61f9a8ff34ea0f4986898104d7e20d25")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerSupervisorState-request)))
  "Returns md5sum for a message object of type 'ControllerSupervisorState-request"
  "61f9a8ff34ea0f4986898104d7e20d25")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerSupervisorState-request>)))
  "Returns full string definition for message of type '<ControllerSupervisorState-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerSupervisorState-request)))
  "Returns full string definition for message of type 'ControllerSupervisorState-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerSupervisorState-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerSupervisorState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerSupervisorState-request
))
;//! \htmlinclude ControllerSupervisorState-response.msg.html

(cl:defclass <ControllerSupervisorState-response> (roslisp-msg-protocol:ros-message)
  ((controller_name
    :reader controller_name
    :initarg :controller_name
    :type cl:string
    :initform "")
   (seq_num
    :reader seq_num
    :initarg :seq_num
    :type cl:integer
    :initform 0))
)

(cl:defclass ControllerSupervisorState-response (<ControllerSupervisorState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerSupervisorState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerSupervisorState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name stoch3_msgs-srv:<ControllerSupervisorState-response> is deprecated: use stoch3_msgs-srv:ControllerSupervisorState-response instead.")))

(cl:ensure-generic-function 'controller_name-val :lambda-list '(m))
(cl:defmethod controller_name-val ((m <ControllerSupervisorState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-srv:controller_name-val is deprecated.  Use stoch3_msgs-srv:controller_name instead.")
  (controller_name m))

(cl:ensure-generic-function 'seq_num-val :lambda-list '(m))
(cl:defmethod seq_num-val ((m <ControllerSupervisorState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-srv:seq_num-val is deprecated.  Use stoch3_msgs-srv:seq_num instead.")
  (seq_num m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerSupervisorState-response>) ostream)
  "Serializes a message object of type '<ControllerSupervisorState-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'controller_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'controller_name))
  (cl:let* ((signed (cl:slot-value msg 'seq_num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerSupervisorState-response>) istream)
  "Deserializes a message object of type '<ControllerSupervisorState-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'controller_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'controller_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'seq_num) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerSupervisorState-response>)))
  "Returns string type for a service object of type '<ControllerSupervisorState-response>"
  "stoch3_msgs/ControllerSupervisorStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerSupervisorState-response)))
  "Returns string type for a service object of type 'ControllerSupervisorState-response"
  "stoch3_msgs/ControllerSupervisorStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerSupervisorState-response>)))
  "Returns md5sum for a message object of type '<ControllerSupervisorState-response>"
  "61f9a8ff34ea0f4986898104d7e20d25")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerSupervisorState-response)))
  "Returns md5sum for a message object of type 'ControllerSupervisorState-response"
  "61f9a8ff34ea0f4986898104d7e20d25")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerSupervisorState-response>)))
  "Returns full string definition for message of type '<ControllerSupervisorState-response>"
  (cl:format cl:nil "string controller_name # Name of the current active controller~%int64  seq_num         # Sequence numer of the current active controller~%                       # (i.e, number of times the controller update function called since it became active)~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerSupervisorState-response)))
  "Returns full string definition for message of type 'ControllerSupervisorState-response"
  (cl:format cl:nil "string controller_name # Name of the current active controller~%int64  seq_num         # Sequence numer of the current active controller~%                       # (i.e, number of times the controller update function called since it became active)~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerSupervisorState-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'controller_name))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerSupervisorState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerSupervisorState-response
    (cl:cons ':controller_name (controller_name msg))
    (cl:cons ':seq_num (seq_num msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ControllerSupervisorState)))
  'ControllerSupervisorState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ControllerSupervisorState)))
  'ControllerSupervisorState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerSupervisorState)))
  "Returns string type for a service object of type '<ControllerSupervisorState>"
  "stoch3_msgs/ControllerSupervisorState")