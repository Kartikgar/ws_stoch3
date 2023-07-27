;; Auto-generated. Do not edit!


(when (boundp 'stoch3_msgs::LegCommandStamped)
  (if (not (find-package "STOCH3_MSGS"))
    (make-package "STOCH3_MSGS"))
  (shadow 'LegCommandStamped (find-package "STOCH3_MSGS")))
(unless (find-package "STOCH3_MSGS::LEGCOMMANDSTAMPED")
  (make-package "STOCH3_MSGS::LEGCOMMANDSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude LegCommandStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass stoch3_msgs::LegCommandStamped
  :super ros::object
  :slots (_header _data ))

(defmethod stoch3_msgs::LegCommandStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:data __data) (instance stoch3_msgs::LegCommand :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _data __data)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:data
   (&rest __data)
   (if (keywordp (car __data))
       (send* _data __data)
     (progn
       (if __data (setq _data (car __data)))
       _data)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; stoch3_msgs/LegCommand _data
    (send _data :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; stoch3_msgs/LegCommand _data
       (send _data :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; stoch3_msgs/LegCommand _data
     (send _data :deserialize buf ptr-) (incf ptr- (send _data :serialization-length))
   ;;
   self)
  )

(setf (get stoch3_msgs::LegCommandStamped :md5sum-) "5604be9aee7bde9f6baa5a2ee1398f28")
(setf (get stoch3_msgs::LegCommandStamped :datatype-) "stoch3_msgs/LegCommandStamped")
(setf (get stoch3_msgs::LegCommandStamped :definition-)
      "Header header
stoch3_msgs/LegCommand data

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
MSG: stoch3_msgs/LegCommand
geometry_msgs/Vector3 position   # Position of foot
geometry_msgs/Vector3 velocity   # Velocity of foot
geometry_msgs/Vector3 force      # Force exerted by foot on environment
float64 kp_scale
float64 kd_scale

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :stoch3_msgs/LegCommandStamped "5604be9aee7bde9f6baa5a2ee1398f28")


