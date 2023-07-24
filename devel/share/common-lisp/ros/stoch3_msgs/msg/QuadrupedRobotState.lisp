; Auto-generated. Do not edit!


(cl:in-package stoch3_msgs-msg)


;//! \htmlinclude QuadrupedRobotState.msg.html

(cl:defclass <QuadrupedRobotState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (twist
    :reader twist
    :initarg :twist
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
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

(cl:defclass QuadrupedRobotState (<QuadrupedRobotState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QuadrupedRobotState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QuadrupedRobotState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name stoch3_msgs-msg:<QuadrupedRobotState> is deprecated: use stoch3_msgs-msg:QuadrupedRobotState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <QuadrupedRobotState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:header-val is deprecated.  Use stoch3_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <QuadrupedRobotState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:pose-val is deprecated.  Use stoch3_msgs-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'twist-val :lambda-list '(m))
(cl:defmethod twist-val ((m <QuadrupedRobotState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:twist-val is deprecated.  Use stoch3_msgs-msg:twist instead.")
  (twist m))

(cl:ensure-generic-function 'fl-val :lambda-list '(m))
(cl:defmethod fl-val ((m <QuadrupedRobotState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:fl-val is deprecated.  Use stoch3_msgs-msg:fl instead.")
  (fl m))

(cl:ensure-generic-function 'fr-val :lambda-list '(m))
(cl:defmethod fr-val ((m <QuadrupedRobotState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:fr-val is deprecated.  Use stoch3_msgs-msg:fr instead.")
  (fr m))

(cl:ensure-generic-function 'bl-val :lambda-list '(m))
(cl:defmethod bl-val ((m <QuadrupedRobotState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:bl-val is deprecated.  Use stoch3_msgs-msg:bl instead.")
  (bl m))

(cl:ensure-generic-function 'br-val :lambda-list '(m))
(cl:defmethod br-val ((m <QuadrupedRobotState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:br-val is deprecated.  Use stoch3_msgs-msg:br instead.")
  (br m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QuadrupedRobotState>) ostream)
  "Serializes a message object of type '<QuadrupedRobotState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'twist) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'fl) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'fr) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'bl) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'br) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QuadrupedRobotState>) istream)
  "Deserializes a message object of type '<QuadrupedRobotState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'twist) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'fl) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'fr) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'bl) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'br) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QuadrupedRobotState>)))
  "Returns string type for a message object of type '<QuadrupedRobotState>"
  "stoch3_msgs/QuadrupedRobotState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QuadrupedRobotState)))
  "Returns string type for a message object of type 'QuadrupedRobotState"
  "stoch3_msgs/QuadrupedRobotState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QuadrupedRobotState>)))
  "Returns md5sum for a message object of type '<QuadrupedRobotState>"
  "c0b1dcdf6adea2ab6ab0d9d2dab7e7de")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QuadrupedRobotState)))
  "Returns md5sum for a message object of type 'QuadrupedRobotState"
  "c0b1dcdf6adea2ab6ab0d9d2dab7e7de")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QuadrupedRobotState>)))
  "Returns full string definition for message of type '<QuadrupedRobotState>"
  (cl:format cl:nil "Header header~%geometry_msgs/Pose pose~%geometry_msgs/Twist twist~%stoch3_msgs/LegState fl~%stoch3_msgs/LegState fr~%stoch3_msgs/LegState bl~%stoch3_msgs/LegState br~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: stoch3_msgs/LegState~%string name~%geometry_msgs/Vector3 position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 force~%float64 support_probability        # [0, 1] , probability that the leg is a support leg.~%                                   # Limit the value to range [0, 1] if it is outside the range.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QuadrupedRobotState)))
  "Returns full string definition for message of type 'QuadrupedRobotState"
  (cl:format cl:nil "Header header~%geometry_msgs/Pose pose~%geometry_msgs/Twist twist~%stoch3_msgs/LegState fl~%stoch3_msgs/LegState fr~%stoch3_msgs/LegState bl~%stoch3_msgs/LegState br~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: stoch3_msgs/LegState~%string name~%geometry_msgs/Vector3 position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 force~%float64 support_probability        # [0, 1] , probability that the leg is a support leg.~%                                   # Limit the value to range [0, 1] if it is outside the range.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QuadrupedRobotState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'twist))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'fl))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'fr))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'bl))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'br))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QuadrupedRobotState>))
  "Converts a ROS message object to a list"
  (cl:list 'QuadrupedRobotState
    (cl:cons ':header (header msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':twist (twist msg))
    (cl:cons ':fl (fl msg))
    (cl:cons ':fr (fr msg))
    (cl:cons ':bl (bl msg))
    (cl:cons ':br (br msg))
))
