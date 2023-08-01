; Auto-generated. Do not edit!


(cl:in-package fovis_msgs-msg)


;//! \htmlinclude VisualOdometryUpdate.msg.html

(cl:defclass <VisualOdometryUpdate> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (curr_timestamp
    :reader curr_timestamp
    :initarg :curr_timestamp
    :type cl:real
    :initform 0)
   (prev_timestamp
    :reader prev_timestamp
    :initarg :prev_timestamp
    :type cl:real
    :initform 0)
   (relative_transform
    :reader relative_transform
    :initarg :relative_transform
    :type geometry_msgs-msg:Transform
    :initform (cl:make-instance 'geometry_msgs-msg:Transform))
   (covariance
    :reader covariance
    :initarg :covariance
    :type (cl:vector cl:float)
   :initform (cl:make-array 36 :element-type 'cl:float :initial-element 0.0))
   (estimate_status
    :reader estimate_status
    :initarg :estimate_status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass VisualOdometryUpdate (<VisualOdometryUpdate>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VisualOdometryUpdate>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VisualOdometryUpdate)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fovis_msgs-msg:<VisualOdometryUpdate> is deprecated: use fovis_msgs-msg:VisualOdometryUpdate instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <VisualOdometryUpdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fovis_msgs-msg:header-val is deprecated.  Use fovis_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'curr_timestamp-val :lambda-list '(m))
(cl:defmethod curr_timestamp-val ((m <VisualOdometryUpdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fovis_msgs-msg:curr_timestamp-val is deprecated.  Use fovis_msgs-msg:curr_timestamp instead.")
  (curr_timestamp m))

(cl:ensure-generic-function 'prev_timestamp-val :lambda-list '(m))
(cl:defmethod prev_timestamp-val ((m <VisualOdometryUpdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fovis_msgs-msg:prev_timestamp-val is deprecated.  Use fovis_msgs-msg:prev_timestamp instead.")
  (prev_timestamp m))

(cl:ensure-generic-function 'relative_transform-val :lambda-list '(m))
(cl:defmethod relative_transform-val ((m <VisualOdometryUpdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fovis_msgs-msg:relative_transform-val is deprecated.  Use fovis_msgs-msg:relative_transform instead.")
  (relative_transform m))

(cl:ensure-generic-function 'covariance-val :lambda-list '(m))
(cl:defmethod covariance-val ((m <VisualOdometryUpdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fovis_msgs-msg:covariance-val is deprecated.  Use fovis_msgs-msg:covariance instead.")
  (covariance m))

(cl:ensure-generic-function 'estimate_status-val :lambda-list '(m))
(cl:defmethod estimate_status-val ((m <VisualOdometryUpdate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fovis_msgs-msg:estimate_status-val is deprecated.  Use fovis_msgs-msg:estimate_status instead.")
  (estimate_status m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<VisualOdometryUpdate>)))
    "Constants for message type '<VisualOdometryUpdate>"
  '((:NO_DATA . 0)
    (:ESTIMATE_VALID . 1)
    (:ESTIMATE_INSUFFICIENT_FEATURES . 2)
    (:ESTIMATE_DEGENERATE . 3)
    (:ESTIMATE_REPROJECTION_ERROR . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'VisualOdometryUpdate)))
    "Constants for message type 'VisualOdometryUpdate"
  '((:NO_DATA . 0)
    (:ESTIMATE_VALID . 1)
    (:ESTIMATE_INSUFFICIENT_FEATURES . 2)
    (:ESTIMATE_DEGENERATE . 3)
    (:ESTIMATE_REPROJECTION_ERROR . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VisualOdometryUpdate>) ostream)
  "Serializes a message object of type '<VisualOdometryUpdate>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'curr_timestamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'curr_timestamp) (cl:floor (cl:slot-value msg 'curr_timestamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'prev_timestamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'prev_timestamp) (cl:floor (cl:slot-value msg 'prev_timestamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'relative_transform) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'covariance))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'estimate_status)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VisualOdometryUpdate>) istream)
  "Deserializes a message object of type '<VisualOdometryUpdate>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'curr_timestamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'prev_timestamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'relative_transform) istream)
  (cl:setf (cl:slot-value msg 'covariance) (cl:make-array 36))
  (cl:let ((vals (cl:slot-value msg 'covariance)))
    (cl:dotimes (i 36)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'estimate_status)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VisualOdometryUpdate>)))
  "Returns string type for a message object of type '<VisualOdometryUpdate>"
  "fovis_msgs/VisualOdometryUpdate")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisualOdometryUpdate)))
  "Returns string type for a message object of type 'VisualOdometryUpdate"
  "fovis_msgs/VisualOdometryUpdate")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VisualOdometryUpdate>)))
  "Returns md5sum for a message object of type '<VisualOdometryUpdate>"
  "86e4e9402396b9789cfeaadd317551b4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VisualOdometryUpdate)))
  "Returns md5sum for a message object of type 'VisualOdometryUpdate"
  "86e4e9402396b9789cfeaadd317551b4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VisualOdometryUpdate>)))
  "Returns full string definition for message of type '<VisualOdometryUpdate>"
  (cl:format cl:nil "Header header~%time curr_timestamp~%time prev_timestamp~%geometry_msgs/Transform relative_transform~%float64[36] covariance~%uint8 estimate_status~%~%uint8 NO_DATA = 0~%uint8 ESTIMATE_VALID = 1~%uint8 ESTIMATE_INSUFFICIENT_FEATURES = 2~%uint8 ESTIMATE_DEGENERATE = 3~%uint8 ESTIMATE_REPROJECTION_ERROR = 4~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VisualOdometryUpdate)))
  "Returns full string definition for message of type 'VisualOdometryUpdate"
  (cl:format cl:nil "Header header~%time curr_timestamp~%time prev_timestamp~%geometry_msgs/Transform relative_transform~%float64[36] covariance~%uint8 estimate_status~%~%uint8 NO_DATA = 0~%uint8 ESTIMATE_VALID = 1~%uint8 ESTIMATE_INSUFFICIENT_FEATURES = 2~%uint8 ESTIMATE_DEGENERATE = 3~%uint8 ESTIMATE_REPROJECTION_ERROR = 4~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VisualOdometryUpdate>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'relative_transform))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'covariance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VisualOdometryUpdate>))
  "Converts a ROS message object to a list"
  (cl:list 'VisualOdometryUpdate
    (cl:cons ':header (header msg))
    (cl:cons ':curr_timestamp (curr_timestamp msg))
    (cl:cons ':prev_timestamp (prev_timestamp msg))
    (cl:cons ':relative_transform (relative_transform msg))
    (cl:cons ':covariance (covariance msg))
    (cl:cons ':estimate_status (estimate_status msg))
))
