; Auto-generated. Do not edit!


(cl:in-package stoch3_msgs-srv)


;//! \htmlinclude Gait-request.msg.html

(cl:defclass <Gait-request> (roslisp-msg-protocol:ros-message)
  ((step_frequency
    :reader step_frequency
    :initarg :step_frequency
    :type cl:float
    :initform 0.0)
   (swing_height
    :reader swing_height
    :initarg :swing_height
    :type cl:float
    :initform 0.0)
   (stance_height
    :reader stance_height
    :initarg :stance_height
    :type cl:float
    :initform 0.0)
   (max_vel
    :reader max_vel
    :initarg :max_vel
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist)))
)

(cl:defclass Gait-request (<Gait-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Gait-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Gait-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name stoch3_msgs-srv:<Gait-request> is deprecated: use stoch3_msgs-srv:Gait-request instead.")))

(cl:ensure-generic-function 'step_frequency-val :lambda-list '(m))
(cl:defmethod step_frequency-val ((m <Gait-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-srv:step_frequency-val is deprecated.  Use stoch3_msgs-srv:step_frequency instead.")
  (step_frequency m))

(cl:ensure-generic-function 'swing_height-val :lambda-list '(m))
(cl:defmethod swing_height-val ((m <Gait-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-srv:swing_height-val is deprecated.  Use stoch3_msgs-srv:swing_height instead.")
  (swing_height m))

(cl:ensure-generic-function 'stance_height-val :lambda-list '(m))
(cl:defmethod stance_height-val ((m <Gait-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-srv:stance_height-val is deprecated.  Use stoch3_msgs-srv:stance_height instead.")
  (stance_height m))

(cl:ensure-generic-function 'max_vel-val :lambda-list '(m))
(cl:defmethod max_vel-val ((m <Gait-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-srv:max_vel-val is deprecated.  Use stoch3_msgs-srv:max_vel instead.")
  (max_vel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Gait-request>) ostream)
  "Serializes a message object of type '<Gait-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'step_frequency))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'swing_height))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'stance_height))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'max_vel) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Gait-request>) istream)
  "Deserializes a message object of type '<Gait-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'step_frequency) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'swing_height) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'stance_height) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'max_vel) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Gait-request>)))
  "Returns string type for a service object of type '<Gait-request>"
  "stoch3_msgs/GaitRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Gait-request)))
  "Returns string type for a service object of type 'Gait-request"
  "stoch3_msgs/GaitRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Gait-request>)))
  "Returns md5sum for a message object of type '<Gait-request>"
  "e17f1b3fe4a600b5b71bcddef43f4905")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Gait-request)))
  "Returns md5sum for a message object of type 'Gait-request"
  "e17f1b3fe4a600b5b71bcddef43f4905")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Gait-request>)))
  "Returns full string definition for message of type '<Gait-request>"
  (cl:format cl:nil "float64 step_frequency~%float64 swing_height~%float64 stance_height~%geometry_msgs/Twist max_vel~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Gait-request)))
  "Returns full string definition for message of type 'Gait-request"
  (cl:format cl:nil "float64 step_frequency~%float64 swing_height~%float64 stance_height~%geometry_msgs/Twist max_vel~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Gait-request>))
  (cl:+ 0
     8
     8
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'max_vel))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Gait-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Gait-request
    (cl:cons ':step_frequency (step_frequency msg))
    (cl:cons ':swing_height (swing_height msg))
    (cl:cons ':stance_height (stance_height msg))
    (cl:cons ':max_vel (max_vel msg))
))
;//! \htmlinclude Gait-response.msg.html

(cl:defclass <Gait-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Gait-response (<Gait-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Gait-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Gait-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name stoch3_msgs-srv:<Gait-response> is deprecated: use stoch3_msgs-srv:Gait-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <Gait-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-srv:ok-val is deprecated.  Use stoch3_msgs-srv:ok instead.")
  (ok m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Gait-response>) ostream)
  "Serializes a message object of type '<Gait-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Gait-response>) istream)
  "Deserializes a message object of type '<Gait-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Gait-response>)))
  "Returns string type for a service object of type '<Gait-response>"
  "stoch3_msgs/GaitResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Gait-response)))
  "Returns string type for a service object of type 'Gait-response"
  "stoch3_msgs/GaitResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Gait-response>)))
  "Returns md5sum for a message object of type '<Gait-response>"
  "e17f1b3fe4a600b5b71bcddef43f4905")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Gait-response)))
  "Returns md5sum for a message object of type 'Gait-response"
  "e17f1b3fe4a600b5b71bcddef43f4905")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Gait-response>)))
  "Returns full string definition for message of type '<Gait-response>"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Gait-response)))
  "Returns full string definition for message of type 'Gait-response"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Gait-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Gait-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Gait-response
    (cl:cons ':ok (ok msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Gait)))
  'Gait-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Gait)))
  'Gait-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Gait)))
  "Returns string type for a service object of type '<Gait>"
  "stoch3_msgs/Gait")