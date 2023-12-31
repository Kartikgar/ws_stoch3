; Auto-generated. Do not edit!


(cl:in-package stoch3_msgs-msg)


;//! \htmlinclude RobotCommand.msg.html

(cl:defclass <RobotCommand> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (twist
    :reader twist
    :initarg :twist
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass RobotCommand (<RobotCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name stoch3_msgs-msg:<RobotCommand> is deprecated: use stoch3_msgs-msg:RobotCommand instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RobotCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:header-val is deprecated.  Use stoch3_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'twist-val :lambda-list '(m))
(cl:defmethod twist-val ((m <RobotCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:twist-val is deprecated.  Use stoch3_msgs-msg:twist instead.")
  (twist m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <RobotCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stoch3_msgs-msg:pose-val is deprecated.  Use stoch3_msgs-msg:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotCommand>) ostream)
  "Serializes a message object of type '<RobotCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'twist) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotCommand>) istream)
  "Deserializes a message object of type '<RobotCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'twist) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotCommand>)))
  "Returns string type for a message object of type '<RobotCommand>"
  "stoch3_msgs/RobotCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotCommand)))
  "Returns string type for a message object of type 'RobotCommand"
  "stoch3_msgs/RobotCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotCommand>)))
  "Returns md5sum for a message object of type '<RobotCommand>"
  "57051507a48caf37604692cb50cc08fe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotCommand)))
  "Returns md5sum for a message object of type 'RobotCommand"
  "57051507a48caf37604692cb50cc08fe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotCommand>)))
  "Returns full string definition for message of type '<RobotCommand>"
  (cl:format cl:nil "Header              header~%geometry_msgs/Twist twist~%geometry_msgs/Pose  pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotCommand)))
  "Returns full string definition for message of type 'RobotCommand"
  (cl:format cl:nil "Header              header~%geometry_msgs/Twist twist~%geometry_msgs/Pose  pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'twist))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotCommand
    (cl:cons ':header (header msg))
    (cl:cons ':twist (twist msg))
    (cl:cons ':pose (pose msg))
))
