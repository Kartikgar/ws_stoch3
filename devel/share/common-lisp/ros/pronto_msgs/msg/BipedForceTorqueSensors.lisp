; Auto-generated. Do not edit!


(cl:in-package pronto_msgs-msg)


;//! \htmlinclude BipedForceTorqueSensors.msg.html

(cl:defclass <BipedForceTorqueSensors> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (l_foot
    :reader l_foot
    :initarg :l_foot
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench))
   (r_foot
    :reader r_foot
    :initarg :r_foot
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench))
   (l_hand
    :reader l_hand
    :initarg :l_hand
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench))
   (r_hand
    :reader r_hand
    :initarg :r_hand
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench)))
)

(cl:defclass BipedForceTorqueSensors (<BipedForceTorqueSensors>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BipedForceTorqueSensors>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BipedForceTorqueSensors)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pronto_msgs-msg:<BipedForceTorqueSensors> is deprecated: use pronto_msgs-msg:BipedForceTorqueSensors instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <BipedForceTorqueSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pronto_msgs-msg:header-val is deprecated.  Use pronto_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'l_foot-val :lambda-list '(m))
(cl:defmethod l_foot-val ((m <BipedForceTorqueSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pronto_msgs-msg:l_foot-val is deprecated.  Use pronto_msgs-msg:l_foot instead.")
  (l_foot m))

(cl:ensure-generic-function 'r_foot-val :lambda-list '(m))
(cl:defmethod r_foot-val ((m <BipedForceTorqueSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pronto_msgs-msg:r_foot-val is deprecated.  Use pronto_msgs-msg:r_foot instead.")
  (r_foot m))

(cl:ensure-generic-function 'l_hand-val :lambda-list '(m))
(cl:defmethod l_hand-val ((m <BipedForceTorqueSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pronto_msgs-msg:l_hand-val is deprecated.  Use pronto_msgs-msg:l_hand instead.")
  (l_hand m))

(cl:ensure-generic-function 'r_hand-val :lambda-list '(m))
(cl:defmethod r_hand-val ((m <BipedForceTorqueSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pronto_msgs-msg:r_hand-val is deprecated.  Use pronto_msgs-msg:r_hand instead.")
  (r_hand m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BipedForceTorqueSensors>) ostream)
  "Serializes a message object of type '<BipedForceTorqueSensors>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'l_foot) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'r_foot) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'l_hand) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'r_hand) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BipedForceTorqueSensors>) istream)
  "Deserializes a message object of type '<BipedForceTorqueSensors>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'l_foot) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'r_foot) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'l_hand) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'r_hand) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BipedForceTorqueSensors>)))
  "Returns string type for a message object of type '<BipedForceTorqueSensors>"
  "pronto_msgs/BipedForceTorqueSensors")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BipedForceTorqueSensors)))
  "Returns string type for a message object of type 'BipedForceTorqueSensors"
  "pronto_msgs/BipedForceTorqueSensors")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BipedForceTorqueSensors>)))
  "Returns md5sum for a message object of type '<BipedForceTorqueSensors>"
  "608f43cae98f77f5fb6cfbbbfa4c9253")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BipedForceTorqueSensors)))
  "Returns md5sum for a message object of type 'BipedForceTorqueSensors"
  "608f43cae98f77f5fb6cfbbbfa4c9253")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BipedForceTorqueSensors>)))
  "Returns full string definition for message of type '<BipedForceTorqueSensors>"
  (cl:format cl:nil "# Atlas force torque sensors for the wrists and ankles~%Header header~%~%geometry_msgs/Wrench l_foot~%geometry_msgs/Wrench r_foot~%geometry_msgs/Wrench l_hand~%geometry_msgs/Wrench r_hand~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BipedForceTorqueSensors)))
  "Returns full string definition for message of type 'BipedForceTorqueSensors"
  (cl:format cl:nil "# Atlas force torque sensors for the wrists and ankles~%Header header~%~%geometry_msgs/Wrench l_foot~%geometry_msgs/Wrench r_foot~%geometry_msgs/Wrench l_hand~%geometry_msgs/Wrench r_hand~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BipedForceTorqueSensors>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'l_foot))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'r_foot))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'l_hand))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'r_hand))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BipedForceTorqueSensors>))
  "Converts a ROS message object to a list"
  (cl:list 'BipedForceTorqueSensors
    (cl:cons ':header (header msg))
    (cl:cons ':l_foot (l_foot msg))
    (cl:cons ':r_foot (r_foot msg))
    (cl:cons ':l_hand (l_hand msg))
    (cl:cons ':r_hand (r_hand msg))
))
