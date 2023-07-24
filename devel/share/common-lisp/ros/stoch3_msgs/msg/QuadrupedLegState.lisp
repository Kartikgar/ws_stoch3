; Auto-generated. Do not edit!


(cl:in-package stoch3_msgs-msg)


;//! \htmlinclude QuadrupedLegState.msg.html

(cl:defclass <QuadrupedLegState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (fl
    :reader fl
    :initarg :fl
    :type stoch3_msgs-msg:LegState
    :initform (cl:make-instance 'stoch3_msgs-msg:LegState))
   (fr
    :reader fr
    :initarg :fr
    :type stoch3_msgs-msg:LegState
    :initform (cl:make-instance 'stoch3_msgs-msg:LegState))
   (bl
    :reader bl
    :initarg :bl
    :type stoch3_msgs-msg:LegState
    :initform (cl:make-instance 'stoch3_msgs-msg:LegState))
   (br
    :reader br
    :initarg :br
    :type stoch3_msgs-msg:LegState
    :initform (cl:make-instance 'stoch3_msgs-msg:LegState)))
)

(cl:defclass QuadrupedLegState (<QuadrupedLegState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QuadrupedLegState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QuadrupedLegState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name stoch3_msgs-msg:<QuadrupedLegState> is deprecated: use stoch3_msgs-msg:QuadrupedLegState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <QuadrupedLegState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:header-val is deprecated.  Use stoch3_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'fl-val :lambda-list '(m))
(cl:defmethod fl-val ((m <QuadrupedLegState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:fl-val is deprecated.  Use stoch3_msgs-msg:fl instead.")
  (fl m))

(cl:ensure-generic-function 'fr-val :lambda-list '(m))
(cl:defmethod fr-val ((m <QuadrupedLegState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:fr-val is deprecated.  Use stoch3_msgs-msg:fr instead.")
  (fr m))

(cl:ensure-generic-function 'bl-val :lambda-list '(m))
(cl:defmethod bl-val ((m <QuadrupedLegState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:bl-val is deprecated.  Use stoch3_msgs-msg:bl instead.")
  (bl m))

(cl:ensure-generic-function 'br-val :lambda-list '(m))
(cl:defmethod br-val ((m <QuadrupedLegState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:br-val is deprecated.  Use stoch3_msgs-msg:br instead.")
  (br m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QuadrupedLegState>) ostream)
  "Serializes a message object of type '<QuadrupedLegState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'fl) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'fr) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'bl) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'br) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QuadrupedLegState>) istream)
  "Deserializes a message object of type '<QuadrupedLegState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'fl) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'fr) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'bl) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'br) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QuadrupedLegState>)))
  "Returns string type for a message object of type '<QuadrupedLegState>"
  "stoch3_msgs/QuadrupedLegState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QuadrupedLegState)))
  "Returns string type for a message object of type 'QuadrupedLegState"
  "stoch3_msgs/QuadrupedLegState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QuadrupedLegState>)))
  "Returns md5sum for a message object of type '<QuadrupedLegState>"
  "bf5e523a21942d2ff068a9be010f7df1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QuadrupedLegState)))
  "Returns md5sum for a message object of type 'QuadrupedLegState"
  "bf5e523a21942d2ff068a9be010f7df1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QuadrupedLegState>)))
  "Returns full string definition for message of type '<QuadrupedLegState>"
  (cl:format cl:nil "Header header~%stoch3_msgs/LegState fl~%stoch3_msgs/LegState fr~%stoch3_msgs/LegState bl~%stoch3_msgs/LegState br~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: stoch3_msgs/LegState~%string name~%geometry_msgs/Vector3 position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 force~%float64 support_probability        # [0, 1] , probability that the leg is a support leg.~%                                   # Limit the value to range [0, 1] if it is outside the range.~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QuadrupedLegState)))
  "Returns full string definition for message of type 'QuadrupedLegState"
  (cl:format cl:nil "Header header~%stoch3_msgs/LegState fl~%stoch3_msgs/LegState fr~%stoch3_msgs/LegState bl~%stoch3_msgs/LegState br~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: stoch3_msgs/LegState~%string name~%geometry_msgs/Vector3 position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 force~%float64 support_probability        # [0, 1] , probability that the leg is a support leg.~%                                   # Limit the value to range [0, 1] if it is outside the range.~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QuadrupedLegState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'fl))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'fr))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'bl))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'br))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QuadrupedLegState>))
  "Converts a ROS message object to a list"
  (cl:list 'QuadrupedLegState
    (cl:cons ':header (header msg))
    (cl:cons ':fl (fl msg))
    (cl:cons ':fr (fr msg))
    (cl:cons ':bl (bl msg))
    (cl:cons ':br (br msg))
))
