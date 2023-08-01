;; Auto-generated. Do not edit!


(when (boundp 'pronto_msgs::QuadrupedForceTorqueSensors)
  (if (not (find-package "PRONTO_MSGS"))
    (make-package "PRONTO_MSGS"))
  (shadow 'QuadrupedForceTorqueSensors (find-package "PRONTO_MSGS")))
(unless (find-package "PRONTO_MSGS::QUADRUPEDFORCETORQUESENSORS")
  (make-package "PRONTO_MSGS::QUADRUPEDFORCETORQUESENSORS"))

(in-package "ROS")
;;//! \htmlinclude QuadrupedForceTorqueSensors.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pronto_msgs::QuadrupedForceTorqueSensors
  :super ros::object
  :slots (_header _lf _rf _lh _rh ))

(defmethod pronto_msgs::QuadrupedForceTorqueSensors
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:lf __lf) (instance geometry_msgs::Wrench :init))
    ((:rf __rf) (instance geometry_msgs::Wrench :init))
    ((:lh __lh) (instance geometry_msgs::Wrench :init))
    ((:rh __rh) (instance geometry_msgs::Wrench :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _lf __lf)
   (setq _rf __rf)
   (setq _lh __lh)
   (setq _rh __rh)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:lf
   (&rest __lf)
   (if (keywordp (car __lf))
       (send* _lf __lf)
     (progn
       (if __lf (setq _lf (car __lf)))
       _lf)))
  (:rf
   (&rest __rf)
   (if (keywordp (car __rf))
       (send* _rf __rf)
     (progn
       (if __rf (setq _rf (car __rf)))
       _rf)))
  (:lh
   (&rest __lh)
   (if (keywordp (car __lh))
       (send* _lh __lh)
     (progn
       (if __lh (setq _lh (car __lh)))
       _lh)))
  (:rh
   (&rest __rh)
   (if (keywordp (car __rh))
       (send* _rh __rh)
     (progn
       (if __rh (setq _rh (car __rh)))
       _rh)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Wrench _lf
    (send _lf :serialization-length)
    ;; geometry_msgs/Wrench _rf
    (send _rf :serialization-length)
    ;; geometry_msgs/Wrench _lh
    (send _lh :serialization-length)
    ;; geometry_msgs/Wrench _rh
    (send _rh :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Wrench _lf
       (send _lf :serialize s)
     ;; geometry_msgs/Wrench _rf
       (send _rf :serialize s)
     ;; geometry_msgs/Wrench _lh
       (send _lh :serialize s)
     ;; geometry_msgs/Wrench _rh
       (send _rh :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Wrench _lf
     (send _lf :deserialize buf ptr-) (incf ptr- (send _lf :serialization-length))
   ;; geometry_msgs/Wrench _rf
     (send _rf :deserialize buf ptr-) (incf ptr- (send _rf :serialization-length))
   ;; geometry_msgs/Wrench _lh
     (send _lh :deserialize buf ptr-) (incf ptr- (send _lh :serialization-length))
   ;; geometry_msgs/Wrench _rh
     (send _rh :deserialize buf ptr-) (incf ptr- (send _rh :serialization-length))
   ;;
   self)
  )

(setf (get pronto_msgs::QuadrupedForceTorqueSensors :md5sum-) "493101afbbcce5b658c33739603c5c3e")
(setf (get pronto_msgs::QuadrupedForceTorqueSensors :datatype-) "pronto_msgs/QuadrupedForceTorqueSensors")
(setf (get pronto_msgs::QuadrupedForceTorqueSensors :definition-)
      "# Atlas force torque sensors for the wrists and ankles
Header header

geometry_msgs/Wrench lf
geometry_msgs/Wrench rf
geometry_msgs/Wrench lh
geometry_msgs/Wrench rh


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



(provide :pronto_msgs/QuadrupedForceTorqueSensors "493101afbbcce5b658c33739603c5c3e")


