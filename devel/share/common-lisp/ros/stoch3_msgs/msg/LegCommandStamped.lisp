; Auto-generated. Do not edit!


(cl:in-package stoch3_msgs-msg)


;//! \htmlinclude LegCommandStamped.msg.html

(cl:defclass <LegCommandStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (data
    :reader data
    :initarg :data
    :type stoch3_msgs-msg:LegCommand
    :initform (cl:make-instance 'stoch3_msgs-msg:LegCommand)))
)

(cl:defclass LegCommandStamped (<LegCommandStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LegCommandStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LegCommandStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name stoch3_msgs-msg:<LegCommandStamped> is deprecated: use stoch3_msgs-msg:LegCommandStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LegCommandStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:header-val is deprecated.  Use stoch3_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <LegCommandStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:data-val is deprecated.  Use stoch3_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LegCommandStamped>) ostream)
  "Serializes a message object of type '<LegCommandStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'data) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LegCommandStamped>) istream)
  "Deserializes a message object of type '<LegCommandStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'data) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LegCommandStamped>)))
  "Returns string type for a message object of type '<LegCommandStamped>"
  "stoch3_msgs/LegCommandStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LegCommandStamped)))
  "Returns string type for a message object of type 'LegCommandStamped"
  "stoch3_msgs/LegCommandStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LegCommandStamped>)))
  "Returns md5sum for a message object of type '<LegCommandStamped>"
  "5604be9aee7bde9f6baa5a2ee1398f28")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LegCommandStamped)))
  "Returns md5sum for a message object of type 'LegCommandStamped"
  "5604be9aee7bde9f6baa5a2ee1398f28")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LegCommandStamped>)))
  "Returns full string definition for message of type '<LegCommandStamped>"
  (cl:format cl:nil "Header header~%stoch3_msgs/LegCommand data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: stoch3_msgs/LegCommand~%geometry_msgs/Vector3 position   # Position of foot~%geometry_msgs/Vector3 velocity   # Velocity of foot~%geometry_msgs/Vector3 force      # Force exerted by foot on environment~%float64 kp_scale~%float64 kd_scale~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LegCommandStamped)))
  "Returns full string definition for message of type 'LegCommandStamped"
  (cl:format cl:nil "Header header~%stoch3_msgs/LegCommand data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: stoch3_msgs/LegCommand~%geometry_msgs/Vector3 position   # Position of foot~%geometry_msgs/Vector3 velocity   # Velocity of foot~%geometry_msgs/Vector3 force      # Force exerted by foot on environment~%float64 kp_scale~%float64 kd_scale~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LegCommandStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LegCommandStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'LegCommandStamped
    (cl:cons ':header (header msg))
    (cl:cons ':data (data msg))
))
