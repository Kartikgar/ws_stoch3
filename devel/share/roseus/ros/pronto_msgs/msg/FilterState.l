;; Auto-generated. Do not edit!


(when (boundp 'pronto_msgs::FilterState)
  (if (not (find-package "PRONTO_MSGS"))
    (make-package "PRONTO_MSGS"))
  (shadow 'FilterState (find-package "PRONTO_MSGS")))
(unless (find-package "PRONTO_MSGS::FILTERSTATE")
  (make-package "PRONTO_MSGS::FILTERSTATE"))

(in-package "ROS")
;;//! \htmlinclude FilterState.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pronto_msgs::FilterState
  :super ros::object
  :slots (_header _quat _state _cov ))

(defmethod pronto_msgs::FilterState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:quat __quat) (instance geometry_msgs::Quaternion :init))
    ((:state __state) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:cov __cov) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _quat __quat)
   (setq _state __state)
   (setq _cov __cov)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:quat
   (&rest __quat)
   (if (keywordp (car __quat))
       (send* _quat __quat)
     (progn
       (if __quat (setq _quat (car __quat)))
       _quat)))
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:cov
   (&optional __cov)
   (if __cov (setq _cov __cov)) _cov)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Quaternion _quat
    (send _quat :serialization-length)
    ;; float64[] _state
    (* 8    (length _state)) 4
    ;; float64[] _cov
    (* 8    (length _cov)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Quaternion _quat
       (send _quat :serialize s)
     ;; float64[] _state
     (write-long (length _state) s)
     (dotimes (i (length _state))
       (sys::poke (elt _state i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _cov
     (write-long (length _cov) s)
     (dotimes (i (length _cov))
       (sys::poke (elt _cov i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Quaternion _quat
     (send _quat :deserialize buf ptr-) (incf ptr- (send _quat :serialization-length))
   ;; float64[] _state
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _state (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _state i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _cov
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cov (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _cov i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get pronto_msgs::FilterState :md5sum-) "be4439d283f2f4408baf08ace05ca824")
(setf (get pronto_msgs::FilterState :datatype-) "pronto_msgs/FilterState")
(setf (get pronto_msgs::FilterState :definition-)
      "Header header
geometry_msgs/Quaternion quat
float64[] state
float64[] cov


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :pronto_msgs/FilterState "be4439d283f2f4408baf08ace05ca824")


