; Auto-generated. Do not edit!


(cl:in-package stoch3_msgs-msg)


;//! \htmlinclude QuadrupedLegCommand.msg.html

(cl:defclass <QuadrupedLegCommand> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (fl
    :reader fl
    :initarg :fl
    :type stoch3_msgs-msg:LegCommand
    :initform (cl:make-instance 'stoch3_msgs-msg:LegCommand))
   (fr
    :reader fr
    :initarg :fr
    :type stoch3_msgs-msg:LegCommand
    :initform (cl:make-instance 'stoch3_msgs-msg:LegCommand))
   (bl
    :reader bl
    :initarg :bl
    :type stoch3_msgs-msg:LegCommand
    :initform (cl:make-instance 'stoch3_msgs-msg:LegCommand))
   (br
    :reader br
    :initarg :br
    :type stoch3_msgs-msg:LegCommand
    :initform (cl:make-instance 'stoch3_msgs-msg:LegCommand)))
)

(cl:defclass QuadrupedLegCommand (<QuadrupedLegCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QuadrupedLegCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QuadrupedLegCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name stoch3_msgs-msg:<QuadrupedLegCommand> is deprecated: use stoch3_msgs-msg:QuadrupedLegCommand instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <QuadrupedLegCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:header-val is deprecated.  Use stoch3_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'fl-val :lambda-list '(m))
(cl:defmethod fl-val ((m <QuadrupedLegCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:fl-val is deprecated.  Use stoch3_msgs-msg:fl instead.")
  (fl m))

(cl:ensure-generic-function 'fr-val :lambda-list '(m))
(cl:defmethod fr-val ((m <QuadrupedLegCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:fr-val is deprecated.  Use stoch3_msgs-msg:fr instead.")
  (fr m))

(cl:ensure-generic-function 'bl-val :lambda-list '(m))
(cl:defmethod bl-val ((m <QuadrupedLegCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:bl-val is deprecated.  Use stoch3_msgs-msg:bl instead.")
  (bl m))

(cl:ensure-generic-function 'br-val :lambda-list '(m))
(cl:defmethod br-val ((m <QuadrupedLegCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:br-val is deprecated.  Use stoch3_msgs-msg:br instead.")
  (br m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QuadrupedLegCommand>) ostream)
  "Serializes a message object of type '<QuadrupedLegCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'fl) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'fr) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'bl) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'br) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QuadrupedLegCommand>) istream)
  "Deserializes a message object of type '<QuadrupedLegCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'fl) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'fr) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'bl) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'br) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QuadrupedLegCommand>)))
  "Returns string type for a message object of type '<QuadrupedLegCommand>"
  "stoch3_msgs/QuadrupedLegCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QuadrupedLegCommand)))
  "Returns string type for a message object of type 'QuadrupedLegCommand"
  "stoch3_msgs/QuadrupedLegCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QuadrupedLegCommand>)))
  "Returns md5sum for a message object of type '<QuadrupedLegCommand>"
  "bfe7d96dc958adfec0e70eda22cb9e0a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QuadrupedLegCommand)))
  "Returns md5sum for a message object of type 'QuadrupedLegCommand"
  "bfe7d96dc958adfec0e70eda22cb9e0a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QuadrupedLegCommand>)))
  "Returns full string definition for message of type '<QuadrupedLegCommand>"
  (cl:format cl:nil "Header    header~%stoch3_msgs/LegCommand fl~%stoch3_msgs/LegCommand fr~%stoch3_msgs/LegCommand bl~%stoch3_msgs/LegCommand br~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: stoch3_msgs/LegCommand~%geometry_msgs/Vector3 position   # Position of foot~%geometry_msgs/Vector3 velocity   # Velocity of foot~%geometry_msgs/Vector3 force      # Force exerted by foot on environment~%float64 kp_scale~%float64 kd_scale~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QuadrupedLegCommand)))
  "Returns full string definition for message of type 'QuadrupedLegCommand"
  (cl:format cl:nil "Header    header~%stoch3_msgs/LegCommand fl~%stoch3_msgs/LegCommand fr~%stoch3_msgs/LegCommand bl~%stoch3_msgs/LegCommand br~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: stoch3_msgs/LegCommand~%geometry_msgs/Vector3 position   # Position of foot~%geometry_msgs/Vector3 velocity   # Velocity of foot~%geometry_msgs/Vector3 force      # Force exerted by foot on environment~%float64 kp_scale~%float64 kd_scale~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QuadrupedLegCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'fl))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'fr))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'bl))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'br))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QuadrupedLegCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'QuadrupedLegCommand
    (cl:cons ':header (header msg))
    (cl:cons ':fl (fl msg))
    (cl:cons ':fr (fr msg))
    (cl:cons ':bl (bl msg))
    (cl:cons ':br (br msg))
))
