; Auto-generated. Do not edit!


(cl:in-package pronto_msgs-msg)


;//! \htmlinclude ControllerFootContact.msg.html

(cl:defclass <ControllerFootContact> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (num_right_foot_contacts
    :reader num_right_foot_contacts
    :initarg :num_right_foot_contacts
    :type cl:integer
    :initform 0)
   (right_foot_contacts
    :reader right_foot_contacts
    :initarg :right_foot_contacts
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (num_left_foot_contacts
    :reader num_left_foot_contacts
    :initarg :num_left_foot_contacts
    :type cl:integer
    :initform 0)
   (left_foot_contacts
    :reader left_foot_contacts
    :initarg :left_foot_contacts
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ControllerFootContact (<ControllerFootContact>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerFootContact>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerFootContact)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pronto_msgs-msg:<ControllerFootContact> is deprecated: use pronto_msgs-msg:ControllerFootContact instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ControllerFootContact>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pronto_msgs-msg:header-val is deprecated.  Use pronto_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'num_right_foot_contacts-val :lambda-list '(m))
(cl:defmethod num_right_foot_contacts-val ((m <ControllerFootContact>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pronto_msgs-msg:num_right_foot_contacts-val is deprecated.  Use pronto_msgs-msg:num_right_foot_contacts instead.")
  (num_right_foot_contacts m))

(cl:ensure-generic-function 'right_foot_contacts-val :lambda-list '(m))
(cl:defmethod right_foot_contacts-val ((m <ControllerFootContact>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pronto_msgs-msg:right_foot_contacts-val is deprecated.  Use pronto_msgs-msg:right_foot_contacts instead.")
  (right_foot_contacts m))

(cl:ensure-generic-function 'num_left_foot_contacts-val :lambda-list '(m))
(cl:defmethod num_left_foot_contacts-val ((m <ControllerFootContact>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pronto_msgs-msg:num_left_foot_contacts-val is deprecated.  Use pronto_msgs-msg:num_left_foot_contacts instead.")
  (num_left_foot_contacts m))

(cl:ensure-generic-function 'left_foot_contacts-val :lambda-list '(m))
(cl:defmethod left_foot_contacts-val ((m <ControllerFootContact>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pronto_msgs-msg:left_foot_contacts-val is deprecated.  Use pronto_msgs-msg:left_foot_contacts instead.")
  (left_foot_contacts m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerFootContact>) ostream)
  "Serializes a message object of type '<ControllerFootContact>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'num_right_foot_contacts)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'right_foot_contacts))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'right_foot_contacts))
  (cl:let* ((signed (cl:slot-value msg 'num_left_foot_contacts)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'left_foot_contacts))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'left_foot_contacts))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerFootContact>) istream)
  "Deserializes a message object of type '<ControllerFootContact>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_right_foot_contacts) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'right_foot_contacts) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'right_foot_contacts)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_left_foot_contacts) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'left_foot_contacts) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'left_foot_contacts)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerFootContact>)))
  "Returns string type for a message object of type '<ControllerFootContact>"
  "pronto_msgs/ControllerFootContact")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerFootContact)))
  "Returns string type for a message object of type 'ControllerFootContact"
  "pronto_msgs/ControllerFootContact")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerFootContact>)))
  "Returns md5sum for a message object of type '<ControllerFootContact>"
  "ac4086dd5e9ba23a62d9963e591cc896")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerFootContact)))
  "Returns md5sum for a message object of type 'ControllerFootContact"
  "ac4086dd5e9ba23a62d9963e591cc896")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerFootContact>)))
  "Returns full string definition for message of type '<ControllerFootContact>"
  (cl:format cl:nil "Header header~%int32 num_right_foot_contacts~%~%# Porting from LCM definition:~%# double right_foot_contacts[3][num_right_foot_contacts]~%float64[] right_foot_contacts~%~%int32 num_left_foot_contacts~%~%# Porting from LCM definition:~%# double left_foot_contacts[3][num_left_foot_contacts];~%float64[] left_foot_contacts~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerFootContact)))
  "Returns full string definition for message of type 'ControllerFootContact"
  (cl:format cl:nil "Header header~%int32 num_right_foot_contacts~%~%# Porting from LCM definition:~%# double right_foot_contacts[3][num_right_foot_contacts]~%float64[] right_foot_contacts~%~%int32 num_left_foot_contacts~%~%# Porting from LCM definition:~%# double left_foot_contacts[3][num_left_foot_contacts];~%float64[] left_foot_contacts~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerFootContact>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'right_foot_contacts) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'left_foot_contacts) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerFootContact>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerFootContact
    (cl:cons ':header (header msg))
    (cl:cons ':num_right_foot_contacts (num_right_foot_contacts msg))
    (cl:cons ':right_foot_contacts (right_foot_contacts msg))
    (cl:cons ':num_left_foot_contacts (num_left_foot_contacts msg))
    (cl:cons ':left_foot_contacts (left_foot_contacts msg))
))
