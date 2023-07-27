;; Auto-generated. Do not edit!


(when (boundp 'stoch3_msgs::QuadrupedLegCommand)
  (if (not (find-package "STOCH3_MSGS"))
    (make-package "STOCH3_MSGS"))
  (shadow 'QuadrupedLegCommand (find-package "STOCH3_MSGS")))
(unless (find-package "STOCH3_MSGS::QUADRUPEDLEGCOMMAND")
  (make-package "STOCH3_MSGS::QUADRUPEDLEGCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude QuadrupedLegCommand.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass stoch3_msgs::QuadrupedLegCommand
  :super ros::object
  :slots (_header _fl _fr _bl _br ))

(defmethod stoch3_msgs::QuadrupedLegCommand
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:fl __fl) (instance stoch3_msgs::LegCommand :init))
    ((:fr __fr) (instance stoch3_msgs::LegCommand :init))
    ((:bl __bl) (instance stoch3_msgs::LegCommand :init))
    ((:br __br) (instance stoch3_msgs::LegCommand :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _fl __fl)
   (setq _fr __fr)
   (setq _bl __bl)
   (setq _br __br)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:fl
   (&rest __fl)
   (if (keywordp (car __fl))
       (send* _fl __fl)
     (progn
       (if __fl (setq _fl (car __fl)))
       _fl)))
  (:fr
   (&rest __fr)
   (if (keywordp (car __fr))
       (send* _fr __fr)
     (progn
       (if __fr (setq _fr (car __fr)))
       _fr)))
  (:bl
   (&rest __bl)
   (if (keywordp (car __bl))
       (send* _bl __bl)
     (progn
       (if __bl (setq _bl (car __bl)))
       _bl)))
  (:br
   (&rest __br)
   (if (keywordp (car __br))
       (send* _br __br)
     (progn
       (if __br (setq _br (car __br)))
       _br)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; stoch3_msgs/LegCommand _fl
    (send _fl :serialization-length)
    ;; stoch3_msgs/LegCommand _fr
    (send _fr :serialization-length)
    ;; stoch3_msgs/LegCommand _bl
    (send _bl :serialization-length)
    ;; stoch3_msgs/LegCommand _br
    (send _br :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; stoch3_msgs/LegCommand _fl
       (send _fl :serialize s)
     ;; stoch3_msgs/LegCommand _fr
       (send _fr :serialize s)
     ;; stoch3_msgs/LegCommand _bl
       (send _bl :serialize s)
     ;; stoch3_msgs/LegCommand _br
       (send _br :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; stoch3_msgs/LegCommand _fl
     (send _fl :deserialize buf ptr-) (incf ptr- (send _fl :serialization-length))
   ;; stoch3_msgs/LegCommand _fr
     (send _fr :deserialize buf ptr-) (incf ptr- (send _fr :serialization-length))
   ;; stoch3_msgs/LegCommand _bl
     (send _bl :deserialize buf ptr-) (incf ptr- (send _bl :serialization-length))
   ;; stoch3_msgs/LegCommand _br
     (send _br :deserialize buf ptr-) (incf ptr- (send _br :serialization-length))
   ;;
   self)
  )

(setf (get stoch3_msgs::QuadrupedLegCommand :md5sum-) "bfe7d96dc958adfec0e70eda22cb9e0a")
(setf (get stoch3_msgs::QuadrupedLegCommand :datatype-) "stoch3_msgs/QuadrupedLegCommand")
(setf (get stoch3_msgs::QuadrupedLegCommand :definition-)
      "Header    header
stoch3_msgs/LegCommand fl
stoch3_msgs/LegCommand fr
stoch3_msgs/LegCommand bl
stoch3_msgs/LegCommand br

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



(provide :stoch3_msgs/QuadrupedLegCommand "bfe7d96dc958adfec0e70eda22cb9e0a")


