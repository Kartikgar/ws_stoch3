; Auto-generated. Do not edit!


(cl:in-package pronto_msgs-msg)


;//! \htmlinclude VelocityWithSigmaBounds.msg.html

(cl:defclass <VelocityWithSigmaBounds> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (velocity_plus_one_sigma
    :reader velocity_plus_one_sigma
    :initarg :velocity_plus_one_sigma
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (velocity_minus_one_sigma
    :reader velocity_minus_one_sigma
    :initarg :velocity_minus_one_sigma
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (plus_one_sigma
    :reader plus_one_sigma
    :initarg :plus_one_sigma
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass VelocityWithSigmaBounds (<VelocityWithSigmaBounds>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VelocityWithSigmaBounds>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VelocityWithSigmaBounds)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pronto_msgs-msg:<VelocityWithSigmaBounds> is deprecated: use pronto_msgs-msg:VelocityWithSigmaBounds instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <VelocityWithSigmaBounds>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pronto_msgs-msg:header-val is deprecated.  Use pronto_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'velocity_plus_one_sigma-val :lambda-list '(m))
(cl:defmethod velocity_plus_one_sigma-val ((m <VelocityWithSigmaBounds>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pronto_msgs-msg:velocity_plus_one_sigma-val is deprecated.  Use pronto_msgs-msg:velocity_plus_one_sigma instead.")
  (velocity_plus_one_sigma m))

(cl:ensure-generic-function 'velocity_minus_one_sigma-val :lambda-list '(m))
(cl:defmethod velocity_minus_one_sigma-val ((m <VelocityWithSigmaBounds>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pronto_msgs-msg:velocity_minus_one_sigma-val is deprecated.  Use pronto_msgs-msg:velocity_minus_one_sigma instead.")
  (velocity_minus_one_sigma m))

(cl:ensure-generic-function 'plus_one_sigma-val :lambda-list '(m))
(cl:defmethod plus_one_sigma-val ((m <VelocityWithSigmaBounds>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pronto_msgs-msg:plus_one_sigma-val is deprecated.  Use pronto_msgs-msg:plus_one_sigma instead.")
  (plus_one_sigma m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VelocityWithSigmaBounds>) ostream)
  "Serializes a message object of type '<VelocityWithSigmaBounds>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity_plus_one_sigma) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity_minus_one_sigma) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'plus_one_sigma) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VelocityWithSigmaBounds>) istream)
  "Deserializes a message object of type '<VelocityWithSigmaBounds>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity_plus_one_sigma) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity_minus_one_sigma) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'plus_one_sigma) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VelocityWithSigmaBounds>)))
  "Returns string type for a message object of type '<VelocityWithSigmaBounds>"
  "pronto_msgs/VelocityWithSigmaBounds")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VelocityWithSigmaBounds)))
  "Returns string type for a message object of type 'VelocityWithSigmaBounds"
  "pronto_msgs/VelocityWithSigmaBounds")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VelocityWithSigmaBounds>)))
  "Returns md5sum for a message object of type '<VelocityWithSigmaBounds>"
  "4f91adbed2b2172ae0f78e162a5bc17d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VelocityWithSigmaBounds)))
  "Returns md5sum for a message object of type 'VelocityWithSigmaBounds"
  "4f91adbed2b2172ae0f78e162a5bc17d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VelocityWithSigmaBounds>)))
  "Returns full string definition for message of type '<VelocityWithSigmaBounds>"
  (cl:format cl:nil "# convenience message encoding the linear velocity~%# one sigma bounds (plus, minus) and the sigma alone~%~%Header header~%geometry_msgs/Vector3 velocity_plus_one_sigma~%geometry_msgs/Vector3 velocity_minus_one_sigma~%geometry_msgs/Vector3 plus_one_sigma~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VelocityWithSigmaBounds)))
  "Returns full string definition for message of type 'VelocityWithSigmaBounds"
  (cl:format cl:nil "# convenience message encoding the linear velocity~%# one sigma bounds (plus, minus) and the sigma alone~%~%Header header~%geometry_msgs/Vector3 velocity_plus_one_sigma~%geometry_msgs/Vector3 velocity_minus_one_sigma~%geometry_msgs/Vector3 plus_one_sigma~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VelocityWithSigmaBounds>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity_plus_one_sigma))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity_minus_one_sigma))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'plus_one_sigma))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VelocityWithSigmaBounds>))
  "Converts a ROS message object to a list"
  (cl:list 'VelocityWithSigmaBounds
    (cl:cons ':header (header msg))
    (cl:cons ':velocity_plus_one_sigma (velocity_plus_one_sigma msg))
    (cl:cons ':velocity_minus_one_sigma (velocity_minus_one_sigma msg))
    (cl:cons ':plus_one_sigma (plus_one_sigma msg))
))
