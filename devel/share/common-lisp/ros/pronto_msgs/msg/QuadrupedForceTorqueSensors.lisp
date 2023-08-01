; Auto-generated. Do not edit!


(cl:in-package pronto_msgs-msg)


;//! \htmlinclude QuadrupedForceTorqueSensors.msg.html

(cl:defclass <QuadrupedForceTorqueSensors> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (lf
    :reader lf
    :initarg :lf
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench))
   (rf
    :reader rf
    :initarg :rf
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench))
   (lh
    :reader lh
    :initarg :lh
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench))
   (rh
    :reader rh
    :initarg :rh
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench)))
)

(cl:defclass QuadrupedForceTorqueSensors (<QuadrupedForceTorqueSensors>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QuadrupedForceTorqueSensors>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QuadrupedForceTorqueSensors)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pronto_msgs-msg:<QuadrupedForceTorqueSensors> is deprecated: use pronto_msgs-msg:QuadrupedForceTorqueSensors instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <QuadrupedForceTorqueSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pronto_msgs-msg:header-val is deprecated.  Use pronto_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'lf-val :lambda-list '(m))
(cl:defmethod lf-val ((m <QuadrupedForceTorqueSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pronto_msgs-msg:lf-val is deprecated.  Use pronto_msgs-msg:lf instead.")
  (lf m))

(cl:ensure-generic-function 'rf-val :lambda-list '(m))
(cl:defmethod rf-val ((m <QuadrupedForceTorqueSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pronto_msgs-msg:rf-val is deprecated.  Use pronto_msgs-msg:rf instead.")
  (rf m))

(cl:ensure-generic-function 'lh-val :lambda-list '(m))
(cl:defmethod lh-val ((m <QuadrupedForceTorqueSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pronto_msgs-msg:lh-val is deprecated.  Use pronto_msgs-msg:lh instead.")
  (lh m))

(cl:ensure-generic-function 'rh-val :lambda-list '(m))
(cl:defmethod rh-val ((m <QuadrupedForceTorqueSensors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pronto_msgs-msg:rh-val is deprecated.  Use pronto_msgs-msg:rh instead.")
  (rh m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QuadrupedForceTorqueSensors>) ostream)
  "Serializes a message object of type '<QuadrupedForceTorqueSensors>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'lf) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'rf) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'lh) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'rh) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QuadrupedForceTorqueSensors>) istream)
  "Deserializes a message object of type '<QuadrupedForceTorqueSensors>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'lf) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'rf) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'lh) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'rh) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QuadrupedForceTorqueSensors>)))
  "Returns string type for a message object of type '<QuadrupedForceTorqueSensors>"
  "pronto_msgs/QuadrupedForceTorqueSensors")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QuadrupedForceTorqueSensors)))
  "Returns string type for a message object of type 'QuadrupedForceTorqueSensors"
  "pronto_msgs/QuadrupedForceTorqueSensors")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QuadrupedForceTorqueSensors>)))
  "Returns md5sum for a message object of type '<QuadrupedForceTorqueSensors>"
  "493101afbbcce5b658c33739603c5c3e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QuadrupedForceTorqueSensors)))
  "Returns md5sum for a message object of type 'QuadrupedForceTorqueSensors"
  "493101afbbcce5b658c33739603c5c3e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QuadrupedForceTorqueSensors>)))
  "Returns full string definition for message of type '<QuadrupedForceTorqueSensors>"
  (cl:format cl:nil "# Atlas force torque sensors for the wrists and ankles~%Header header~%~%geometry_msgs/Wrench lf~%geometry_msgs/Wrench rf~%geometry_msgs/Wrench lh~%geometry_msgs/Wrench rh~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QuadrupedForceTorqueSensors)))
  "Returns full string definition for message of type 'QuadrupedForceTorqueSensors"
  (cl:format cl:nil "# Atlas force torque sensors for the wrists and ankles~%Header header~%~%geometry_msgs/Wrench lf~%geometry_msgs/Wrench rf~%geometry_msgs/Wrench lh~%geometry_msgs/Wrench rh~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QuadrupedForceTorqueSensors>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'lf))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'rf))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'lh))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'rh))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QuadrupedForceTorqueSensors>))
  "Converts a ROS message object to a list"
  (cl:list 'QuadrupedForceTorqueSensors
    (cl:cons ':header (header msg))
    (cl:cons ':lf (lf msg))
    (cl:cons ':rf (rf msg))
    (cl:cons ':lh (lh msg))
    (cl:cons ':rh (rh msg))
))
