; Auto-generated. Do not edit!


(cl:in-package stoch3_msgs-msg)


;//! \htmlinclude SitStandResult.msg.html

(cl:defclass <SitStandResult> (roslisp-msg-protocol:ros-message)
  ((progress
    :reader progress
    :initarg :progress
    :type cl:float
    :initform 0.0))
)

(cl:defclass SitStandResult (<SitStandResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SitStandResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SitStandResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name stoch3_msgs-msg:<SitStandResult> is deprecated: use stoch3_msgs-msg:SitStandResult instead.")))

(cl:ensure-generic-function 'progress-val :lambda-list '(m))
(cl:defmethod progress-val ((m <SitStandResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:progress-val is deprecated.  Use stoch3_msgs-msg:progress instead.")
  (progress m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SitStandResult>) ostream)
  "Serializes a message object of type '<SitStandResult>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'progress))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SitStandResult>) istream)
  "Deserializes a message object of type '<SitStandResult>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'progress) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SitStandResult>)))
  "Returns string type for a message object of type '<SitStandResult>"
  "stoch3_msgs/SitStandResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SitStandResult)))
  "Returns string type for a message object of type 'SitStandResult"
  "stoch3_msgs/SitStandResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SitStandResult>)))
  "Returns md5sum for a message object of type '<SitStandResult>"
  "70805092fd20e110765c7b79e8efb737")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SitStandResult)))
  "Returns md5sum for a message object of type 'SitStandResult"
  "70805092fd20e110765c7b79e8efb737")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SitStandResult>)))
  "Returns full string definition for message of type '<SitStandResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%float32 progress~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SitStandResult)))
  "Returns full string definition for message of type 'SitStandResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%float32 progress~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SitStandResult>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SitStandResult>))
  "Converts a ROS message object to a list"
  (cl:list 'SitStandResult
    (cl:cons ':progress (progress msg))
))
