; Auto-generated. Do not edit!


(cl:in-package fovis_msgs-msg)


;//! \htmlinclude Stats.msg.html

(cl:defclass <Stats> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (num_matches
    :reader num_matches
    :initarg :num_matches
    :type cl:integer
    :initform 0)
   (num_inliers
    :reader num_inliers
    :initarg :num_inliers
    :type cl:integer
    :initform 0)
   (num_detected_keypoints
    :reader num_detected_keypoints
    :initarg :num_detected_keypoints
    :type cl:integer
    :initform 0)
   (num_reprojection_failures
    :reader num_reprojection_failures
    :initarg :num_reprojection_failures
    :type cl:integer
    :initform 0)
   (num_keypoints
    :reader num_keypoints
    :initarg :num_keypoints
    :type cl:integer
    :initform 0)
   (mean_reprojection_error
    :reader mean_reprojection_error
    :initarg :mean_reprojection_error
    :type cl:float
    :initform 0.0)
   (fast_threshold
    :reader fast_threshold
    :initarg :fast_threshold
    :type cl:float
    :initform 0.0)
   (status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Stats (<Stats>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Stats>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Stats)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fovis_msgs-msg:<Stats> is deprecated: use fovis_msgs-msg:Stats instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Stats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fovis_msgs-msg:header-val is deprecated.  Use fovis_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'num_matches-val :lambda-list '(m))
(cl:defmethod num_matches-val ((m <Stats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fovis_msgs-msg:num_matches-val is deprecated.  Use fovis_msgs-msg:num_matches instead.")
  (num_matches m))

(cl:ensure-generic-function 'num_inliers-val :lambda-list '(m))
(cl:defmethod num_inliers-val ((m <Stats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fovis_msgs-msg:num_inliers-val is deprecated.  Use fovis_msgs-msg:num_inliers instead.")
  (num_inliers m))

(cl:ensure-generic-function 'num_detected_keypoints-val :lambda-list '(m))
(cl:defmethod num_detected_keypoints-val ((m <Stats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fovis_msgs-msg:num_detected_keypoints-val is deprecated.  Use fovis_msgs-msg:num_detected_keypoints instead.")
  (num_detected_keypoints m))

(cl:ensure-generic-function 'num_reprojection_failures-val :lambda-list '(m))
(cl:defmethod num_reprojection_failures-val ((m <Stats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fovis_msgs-msg:num_reprojection_failures-val is deprecated.  Use fovis_msgs-msg:num_reprojection_failures instead.")
  (num_reprojection_failures m))

(cl:ensure-generic-function 'num_keypoints-val :lambda-list '(m))
(cl:defmethod num_keypoints-val ((m <Stats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fovis_msgs-msg:num_keypoints-val is deprecated.  Use fovis_msgs-msg:num_keypoints instead.")
  (num_keypoints m))

(cl:ensure-generic-function 'mean_reprojection_error-val :lambda-list '(m))
(cl:defmethod mean_reprojection_error-val ((m <Stats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fovis_msgs-msg:mean_reprojection_error-val is deprecated.  Use fovis_msgs-msg:mean_reprojection_error instead.")
  (mean_reprojection_error m))

(cl:ensure-generic-function 'fast_threshold-val :lambda-list '(m))
(cl:defmethod fast_threshold-val ((m <Stats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fovis_msgs-msg:fast_threshold-val is deprecated.  Use fovis_msgs-msg:fast_threshold instead.")
  (fast_threshold m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <Stats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fovis_msgs-msg:status-val is deprecated.  Use fovis_msgs-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Stats>)))
    "Constants for message type '<Stats>"
  '((:STATUS_CODE_NO_DATA . 0)
    (:STATUS_CODE_SUCCESS . 1)
    (:STATUS_CODE_INSUFFICIENT_INLIERS . 2)
    (:STATUS_CODE_OPTIMIZATION_FAILURE . 3)
    (:STATUS_CODE_REPROJECTION_ERROR . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Stats)))
    "Constants for message type 'Stats"
  '((:STATUS_CODE_NO_DATA . 0)
    (:STATUS_CODE_SUCCESS . 1)
    (:STATUS_CODE_INSUFFICIENT_INLIERS . 2)
    (:STATUS_CODE_OPTIMIZATION_FAILURE . 3)
    (:STATUS_CODE_REPROJECTION_ERROR . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Stats>) ostream)
  "Serializes a message object of type '<Stats>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'num_matches)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'num_inliers)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'num_detected_keypoints)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'num_reprojection_failures)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'num_keypoints)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'mean_reprojection_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'fast_threshold))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Stats>) istream)
  "Deserializes a message object of type '<Stats>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_matches) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_inliers) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_detected_keypoints) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_reprojection_failures) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_keypoints) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mean_reprojection_error) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fast_threshold) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Stats>)))
  "Returns string type for a message object of type '<Stats>"
  "fovis_msgs/Stats")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Stats)))
  "Returns string type for a message object of type 'Stats"
  "fovis_msgs/Stats")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Stats>)))
  "Returns md5sum for a message object of type '<Stats>"
  "dbaf83c8b3afd2954f468abc4e36c223")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Stats)))
  "Returns md5sum for a message object of type 'Stats"
  "dbaf83c8b3afd2954f468abc4e36c223")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Stats>)))
  "Returns full string definition for message of type '<Stats>"
  (cl:format cl:nil "# state of fovis~%# same as fovis::stats_t lcm message~%Header header~%~%int32 num_matches~%int32 num_inliers~%int32 num_detected_keypoints~%int32 num_reprojection_failures~%~%# keypoints after pruning~%int32 num_keypoints~%~%float64 mean_reprojection_error~%float64 fast_threshold~%~%uint8 STATUS_CODE_NO_DATA = 0~%uint8 STATUS_CODE_SUCCESS = 1~%uint8 STATUS_CODE_INSUFFICIENT_INLIERS = 2~%uint8 STATUS_CODE_OPTIMIZATION_FAILURE = 3~%uint8 STATUS_CODE_REPROJECTION_ERROR = 4~%~%uint8 status~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Stats)))
  "Returns full string definition for message of type 'Stats"
  (cl:format cl:nil "# state of fovis~%# same as fovis::stats_t lcm message~%Header header~%~%int32 num_matches~%int32 num_inliers~%int32 num_detected_keypoints~%int32 num_reprojection_failures~%~%# keypoints after pruning~%int32 num_keypoints~%~%float64 mean_reprojection_error~%float64 fast_threshold~%~%uint8 STATUS_CODE_NO_DATA = 0~%uint8 STATUS_CODE_SUCCESS = 1~%uint8 STATUS_CODE_INSUFFICIENT_INLIERS = 2~%uint8 STATUS_CODE_OPTIMIZATION_FAILURE = 3~%uint8 STATUS_CODE_REPROJECTION_ERROR = 4~%~%uint8 status~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Stats>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
     8
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Stats>))
  "Converts a ROS message object to a list"
  (cl:list 'Stats
    (cl:cons ':header (header msg))
    (cl:cons ':num_matches (num_matches msg))
    (cl:cons ':num_inliers (num_inliers msg))
    (cl:cons ':num_detected_keypoints (num_detected_keypoints msg))
    (cl:cons ':num_reprojection_failures (num_reprojection_failures msg))
    (cl:cons ':num_keypoints (num_keypoints msg))
    (cl:cons ':mean_reprojection_error (mean_reprojection_error msg))
    (cl:cons ':fast_threshold (fast_threshold msg))
    (cl:cons ':status (status msg))
))
