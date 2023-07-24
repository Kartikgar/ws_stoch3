; Auto-generated. Do not edit!


(cl:in-package stoch3_msgs-msg)


;//! \htmlinclude LegCommand.msg.html

(cl:defclass <LegCommand> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (velocity
    :reader velocity
    :initarg :velocity
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (force
    :reader force
    :initarg :force
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (kp_scale
    :reader kp_scale
    :initarg :kp_scale
    :type cl:float
    :initform 0.0)
   (kd_scale
    :reader kd_scale
    :initarg :kd_scale
    :type cl:float
    :initform 0.0))
)

(cl:defclass LegCommand (<LegCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LegCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LegCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name stoch3_msgs-msg:<LegCommand> is deprecated: use stoch3_msgs-msg:LegCommand instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <LegCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:position-val is deprecated.  Use stoch3_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <LegCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:velocity-val is deprecated.  Use stoch3_msgs-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'force-val :lambda-list '(m))
(cl:defmethod force-val ((m <LegCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:force-val is deprecated.  Use stoch3_msgs-msg:force instead.")
  (force m))

(cl:ensure-generic-function 'kp_scale-val :lambda-list '(m))
(cl:defmethod kp_scale-val ((m <LegCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:kp_scale-val is deprecated.  Use stoch3_msgs-msg:kp_scale instead.")
  (kp_scale m))

(cl:ensure-generic-function 'kd_scale-val :lambda-list '(m))
(cl:defmethod kd_scale-val ((m <LegCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:kd_scale-val is deprecated.  Use stoch3_msgs-msg:kd_scale instead.")
  (kd_scale m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LegCommand>) ostream)
  "Serializes a message object of type '<LegCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'force) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'kp_scale))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'kd_scale))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LegCommand>) istream)
  "Deserializes a message object of type '<LegCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'force) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'kp_scale) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'kd_scale) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LegCommand>)))
  "Returns string type for a message object of type '<LegCommand>"
  "stoch3_msgs/LegCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LegCommand)))
  "Returns string type for a message object of type 'LegCommand"
  "stoch3_msgs/LegCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LegCommand>)))
  "Returns md5sum for a message object of type '<LegCommand>"
  "ecee7b804ccb7dfdc74daea16cd92c3f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LegCommand)))
  "Returns md5sum for a message object of type 'LegCommand"
  "ecee7b804ccb7dfdc74daea16cd92c3f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LegCommand>)))
  "Returns full string definition for message of type '<LegCommand>"
  (cl:format cl:nil "geometry_msgs/Vector3 position   # Position of foot~%geometry_msgs/Vector3 velocity   # Velocity of foot~%geometry_msgs/Vector3 force      # Force exerted by foot on environment~%float64 kp_scale~%float64 kd_scale~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LegCommand)))
  "Returns full string definition for message of type 'LegCommand"
  (cl:format cl:nil "geometry_msgs/Vector3 position   # Position of foot~%geometry_msgs/Vector3 velocity   # Velocity of foot~%geometry_msgs/Vector3 force      # Force exerted by foot on environment~%float64 kp_scale~%float64 kd_scale~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LegCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'force))
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LegCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'LegCommand
    (cl:cons ':position (position msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':force (force msg))
    (cl:cons ':kp_scale (kp_scale msg))
    (cl:cons ':kd_scale (kd_scale msg))
))
