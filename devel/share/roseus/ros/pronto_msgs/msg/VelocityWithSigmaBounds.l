;; Auto-generated. Do not edit!


(when (boundp 'pronto_msgs::VelocityWithSigmaBounds)
  (if (not (find-package "PRONTO_MSGS"))
    (make-package "PRONTO_MSGS"))
  (shadow 'VelocityWithSigmaBounds (find-package "PRONTO_MSGS")))
(unless (find-package "PRONTO_MSGS::VELOCITYWITHSIGMABOUNDS")
  (make-package "PRONTO_MSGS::VELOCITYWITHSIGMABOUNDS"))

(in-package "ROS")
;;//! \htmlinclude VelocityWithSigmaBounds.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pronto_msgs::VelocityWithSigmaBounds
  :super ros::object
  :slots (_header _velocity_plus_one_sigma _velocity_minus_one_sigma _plus_one_sigma ))

(defmethod pronto_msgs::VelocityWithSigmaBounds
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:velocity_plus_one_sigma __velocity_plus_one_sigma) (instance geometry_msgs::Vector3 :init))
    ((:velocity_minus_one_sigma __velocity_minus_one_sigma) (instance geometry_msgs::Vector3 :init))
    ((:plus_one_sigma __plus_one_sigma) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _velocity_plus_one_sigma __velocity_plus_one_sigma)
   (setq _velocity_minus_one_sigma __velocity_minus_one_sigma)
   (setq _plus_one_sigma __plus_one_sigma)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:velocity_plus_one_sigma
   (&rest __velocity_plus_one_sigma)
   (if (keywordp (car __velocity_plus_one_sigma))
       (send* _velocity_plus_one_sigma __velocity_plus_one_sigma)
     (progn
       (if __velocity_plus_one_sigma (setq _velocity_plus_one_sigma (car __velocity_plus_one_sigma)))
       _velocity_plus_one_sigma)))
  (:velocity_minus_one_sigma
   (&rest __velocity_minus_one_sigma)
   (if (keywordp (car __velocity_minus_one_sigma))
       (send* _velocity_minus_one_sigma __velocity_minus_one_sigma)
     (progn
       (if __velocity_minus_one_sigma (setq _velocity_minus_one_sigma (car __velocity_minus_one_sigma)))
       _velocity_minus_one_sigma)))
  (:plus_one_sigma
   (&rest __plus_one_sigma)
   (if (keywordp (car __plus_one_sigma))
       (send* _plus_one_sigma __plus_one_sigma)
     (progn
       (if __plus_one_sigma (setq _plus_one_sigma (car __plus_one_sigma)))
       _plus_one_sigma)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Vector3 _velocity_plus_one_sigma
    (send _velocity_plus_one_sigma :serialization-length)
    ;; geometry_msgs/Vector3 _velocity_minus_one_sigma
    (send _velocity_minus_one_sigma :serialization-length)
    ;; geometry_msgs/Vector3 _plus_one_sigma
    (send _plus_one_sigma :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Vector3 _velocity_plus_one_sigma
       (send _velocity_plus_one_sigma :serialize s)
     ;; geometry_msgs/Vector3 _velocity_minus_one_sigma
       (send _velocity_minus_one_sigma :serialize s)
     ;; geometry_msgs/Vector3 _plus_one_sigma
       (send _plus_one_sigma :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Vector3 _velocity_plus_one_sigma
     (send _velocity_plus_one_sigma :deserialize buf ptr-) (incf ptr- (send _velocity_plus_one_sigma :serialization-length))
   ;; geometry_msgs/Vector3 _velocity_minus_one_sigma
     (send _velocity_minus_one_sigma :deserialize buf ptr-) (incf ptr- (send _velocity_minus_one_sigma :serialization-length))
   ;; geometry_msgs/Vector3 _plus_one_sigma
     (send _plus_one_sigma :deserialize buf ptr-) (incf ptr- (send _plus_one_sigma :serialization-length))
   ;;
   self)
  )

(setf (get pronto_msgs::VelocityWithSigmaBounds :md5sum-) "4f91adbed2b2172ae0f78e162a5bc17d")
(setf (get pronto_msgs::VelocityWithSigmaBounds :datatype-) "pronto_msgs/VelocityWithSigmaBounds")
(setf (get pronto_msgs::VelocityWithSigmaBounds :definition-)
      "# convenience message encoding the linear velocity
# one sigma bounds (plus, minus) and the sigma alone

Header header
geometry_msgs/Vector3 velocity_plus_one_sigma
geometry_msgs/Vector3 velocity_minus_one_sigma
geometry_msgs/Vector3 plus_one_sigma


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



(provide :pronto_msgs/VelocityWithSigmaBounds "4f91adbed2b2172ae0f78e162a5bc17d")


