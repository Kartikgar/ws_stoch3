;; Auto-generated. Do not edit!


(when (boundp 'pronto_msgs::BipedForceTorqueSensors)
  (if (not (find-package "PRONTO_MSGS"))
    (make-package "PRONTO_MSGS"))
  (shadow 'BipedForceTorqueSensors (find-package "PRONTO_MSGS")))
(unless (find-package "PRONTO_MSGS::BIPEDFORCETORQUESENSORS")
  (make-package "PRONTO_MSGS::BIPEDFORCETORQUESENSORS"))

(in-package "ROS")
;;//! \htmlinclude BipedForceTorqueSensors.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pronto_msgs::BipedForceTorqueSensors
  :super ros::object
  :slots (_header _l_foot _r_foot _l_hand _r_hand ))

(defmethod pronto_msgs::BipedForceTorqueSensors
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:l_foot __l_foot) (instance geometry_msgs::Wrench :init))
    ((:r_foot __r_foot) (instance geometry_msgs::Wrench :init))
    ((:l_hand __l_hand) (instance geometry_msgs::Wrench :init))
    ((:r_hand __r_hand) (instance geometry_msgs::Wrench :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _l_foot __l_foot)
   (setq _r_foot __r_foot)
   (setq _l_hand __l_hand)
   (setq _r_hand __r_hand)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:l_foot
   (&rest __l_foot)
   (if (keywordp (car __l_foot))
       (send* _l_foot __l_foot)
     (progn
       (if __l_foot (setq _l_foot (car __l_foot)))
       _l_foot)))
  (:r_foot
   (&rest __r_foot)
   (if (keywordp (car __r_foot))
       (send* _r_foot __r_foot)
     (progn
       (if __r_foot (setq _r_foot (car __r_foot)))
       _r_foot)))
  (:l_hand
   (&rest __l_hand)
   (if (keywordp (car __l_hand))
       (send* _l_hand __l_hand)
     (progn
       (if __l_hand (setq _l_hand (car __l_hand)))
       _l_hand)))
  (:r_hand
   (&rest __r_hand)
   (if (keywordp (car __r_hand))
       (send* _r_hand __r_hand)
     (progn
       (if __r_hand (setq _r_hand (car __r_hand)))
       _r_hand)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Wrench _l_foot
    (send _l_foot :serialization-length)
    ;; geometry_msgs/Wrench _r_foot
    (send _r_foot :serialization-length)
    ;; geometry_msgs/Wrench _l_hand
    (send _l_hand :serialization-length)
    ;; geometry_msgs/Wrench _r_hand
    (send _r_hand :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Wrench _l_foot
       (send _l_foot :serialize s)
     ;; geometry_msgs/Wrench _r_foot
       (send _r_foot :serialize s)
     ;; geometry_msgs/Wrench _l_hand
       (send _l_hand :serialize s)
     ;; geometry_msgs/Wrench _r_hand
       (send _r_hand :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Wrench _l_foot
     (send _l_foot :deserialize buf ptr-) (incf ptr- (send _l_foot :serialization-length))
   ;; geometry_msgs/Wrench _r_foot
     (send _r_foot :deserialize buf ptr-) (incf ptr- (send _r_foot :serialization-length))
   ;; geometry_msgs/Wrench _l_hand
     (send _l_hand :deserialize buf ptr-) (incf ptr- (send _l_hand :serialization-length))
   ;; geometry_msgs/Wrench _r_hand
     (send _r_hand :deserialize buf ptr-) (incf ptr- (send _r_hand :serialization-length))
   ;;
   self)
  )

(setf (get pronto_msgs::BipedForceTorqueSensors :md5sum-) "608f43cae98f77f5fb6cfbbbfa4c9253")
(setf (get pronto_msgs::BipedForceTorqueSensors :datatype-) "pronto_msgs/BipedForceTorqueSensors")
(setf (get pronto_msgs::BipedForceTorqueSensors :definition-)
      "# Atlas force torque sensors for the wrists and ankles
Header header

geometry_msgs/Wrench l_foot
geometry_msgs/Wrench r_foot
geometry_msgs/Wrench l_hand
geometry_msgs/Wrench r_hand

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
MSG: geometry_msgs/Wrench
# This represents force in free space, separated into
# its linear and angular parts.
Vector3  force
Vector3  torque

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



(provide :pronto_msgs/BipedForceTorqueSensors "608f43cae98f77f5fb6cfbbbfa4c9253")


