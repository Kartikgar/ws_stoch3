; Auto-generated. Do not edit!


(cl:in-package stoch3_msgs-msg)


;//! \htmlinclude LegStateStamped.msg.html

(cl:defclass <LegStateStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (data
    :reader data
    :initarg :data
    :type stoch3_msgs-msg:LegState
    :initform (cl:make-instance 'stoch3_msgs-msg:LegState)))
)

(cl:defclass LegStateStamped (<LegStateStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LegStateStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LegStateStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name stoch3_msgs-msg:<LegStateStamped> is deprecated: use stoch3_msgs-msg:LegStateStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LegStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:header-val is deprecated.  Use stoch3_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <LegStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:data-val is deprecated.  Use stoch3_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LegStateStamped>) ostream)
  "Serializes a message object of type '<LegStateStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'data) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LegStateStamped>) istream)
  "Deserializes a message object of type '<LegStateStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'data) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LegStateStamped>)))
  "Returns string type for a message object of type '<LegStateStamped>"
  "stoch3_msgs/LegStateStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LegStateStamped)))
  "Returns string type for a message object of type 'LegStateStamped"
  "stoch3_msgs/LegStateStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LegStateStamped>)))
  "Returns md5sum for a message object of type '<LegStateStamped>"
  "b8a8ec9ab64dc6e73badf36e24479af3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LegStateStamped)))
  "Returns md5sum for a message object of type 'LegStateStamped"
  "b8a8ec9ab64dc6e73badf36e24479af3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LegStateStamped>)))
  "Returns full string definition for message of type '<LegStateStamped>"
  (cl:format cl:nil "Header header~%stoch3_msgs/LegState data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: stoch3_msgs/LegState~%string name~%geometry_msgs/Vector3 position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 force~%float64 support_probability        # [0, 1] , probability that the leg is a support leg.~%                                   # Limit the value to range [0, 1] if it is outside the range.~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LegStateStamped)))
  "Returns full string definition for message of type 'LegStateStamped"
  (cl:format cl:nil "Header header~%stoch3_msgs/LegState data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: stoch3_msgs/LegState~%string name~%geometry_msgs/Vector3 position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 force~%float64 support_probability        # [0, 1] , probability that the leg is a support leg.~%                                   # Limit the value to range [0, 1] if it is outside the range.~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LegStateStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LegStateStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'LegStateStamped
    (cl:cons ':header (header msg))
    (cl:cons ':data (data msg))
))
