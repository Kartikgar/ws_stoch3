; Auto-generated. Do not edit!


(cl:in-package stoch3_msgs-srv)


;//! \htmlinclude SwitchController-request.msg.html

(cl:defclass <SwitchController-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass SwitchController-request (<SwitchController-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SwitchController-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SwitchController-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name stoch3_msgs-srv:<SwitchController-request> is deprecated: use stoch3_msgs-srv:SwitchController-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <SwitchController-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-srv:name-val is deprecated.  Use stoch3_msgs-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SwitchController-request>) ostream)
  "Serializes a message object of type '<SwitchController-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SwitchController-request>) istream)
  "Deserializes a message object of type '<SwitchController-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SwitchController-request>)))
  "Returns string type for a service object of type '<SwitchController-request>"
  "stoch3_msgs/SwitchControllerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SwitchController-request)))
  "Returns string type for a service object of type 'SwitchController-request"
  "stoch3_msgs/SwitchControllerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SwitchController-request>)))
  "Returns md5sum for a message object of type '<SwitchController-request>"
  "647e5c54b8d6468952d8d31f1efe96c0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SwitchController-request)))
  "Returns md5sum for a message object of type 'SwitchController-request"
  "647e5c54b8d6468952d8d31f1efe96c0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SwitchController-request>)))
  "Returns full string definition for message of type '<SwitchController-request>"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SwitchController-request)))
  "Returns full string definition for message of type 'SwitchController-request"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SwitchController-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SwitchController-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SwitchController-request
    (cl:cons ':name (name msg))
))
;//! \htmlinclude SwitchController-response.msg.html

(cl:defclass <SwitchController-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SwitchController-response (<SwitchController-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SwitchController-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SwitchController-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name stoch3_msgs-srv:<SwitchController-response> is deprecated: use stoch3_msgs-srv:SwitchController-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <SwitchController-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-srv:ok-val is deprecated.  Use stoch3_msgs-srv:ok instead.")
  (ok m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SwitchController-response>) ostream)
  "Serializes a message object of type '<SwitchController-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SwitchController-response>) istream)
  "Deserializes a message object of type '<SwitchController-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SwitchController-response>)))
  "Returns string type for a service object of type '<SwitchController-response>"
  "stoch3_msgs/SwitchControllerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SwitchController-response)))
  "Returns string type for a service object of type 'SwitchController-response"
  "stoch3_msgs/SwitchControllerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SwitchController-response>)))
  "Returns md5sum for a message object of type '<SwitchController-response>"
  "647e5c54b8d6468952d8d31f1efe96c0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SwitchController-response)))
  "Returns md5sum for a message object of type 'SwitchController-response"
  "647e5c54b8d6468952d8d31f1efe96c0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SwitchController-response>)))
  "Returns full string definition for message of type '<SwitchController-response>"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SwitchController-response)))
  "Returns full string definition for message of type 'SwitchController-response"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SwitchController-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SwitchController-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SwitchController-response
    (cl:cons ':ok (ok msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SwitchController)))
  'SwitchController-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SwitchController)))
  'SwitchController-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SwitchController)))
  "Returns string type for a service object of type '<SwitchController>"
  "stoch3_msgs/SwitchController")