;; Auto-generated. Do not edit!


(when (boundp 'stoch3_msgs::QuadrupedRobotState)
  (if (not (find-package "STOCH3_MSGS"))
    (make-package "STOCH3_MSGS"))
  (shadow 'QuadrupedRobotState (find-package "STOCH3_MSGS")))
(unless (find-package "STOCH3_MSGS::QUADRUPEDROBOTSTATE")
  (make-package "STOCH3_MSGS::QUADRUPEDROBOTSTATE"))

(in-package "ROS")
;;//! \htmlinclude QuadrupedRobotState.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass stoch3_msgs::QuadrupedRobotState
  :super ros::object
  :slots (_header _pose _twist _fl _fr _bl _br ))

(defmethod stoch3_msgs::QuadrupedRobotState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:twist __twist) (instance geometry_msgs::Twist :init))
    ((:fl __fl) (instance stoch3_msgs::LegState :init))
    ((:fr __fr) (instance stoch3_msgs::LegState :init))
    ((:bl __bl) (instance stoch3_msgs::LegState :init))
    ((:br __br) (instance stoch3_msgs::LegState :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _pose __pose)
   (setq _twist __twist)
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
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:twist
   (&rest __twist)
   (if (keywordp (car __twist))
       (send* _twist __twist)
     (progn
       (if __twist (setq _twist (car __twist)))
       _twist)))
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
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; geometry_msgs/Twist _twist
    (send _twist :serialization-length)
    ;; stoch3_msgs/LegState _fl
    (send _fl :serialization-length)
    ;; stoch3_msgs/LegState _fr
    (send _fr :serialization-length)
    ;; stoch3_msgs/LegState _bl
    (send _bl :serialization-length)
    ;; stoch3_msgs/LegState _br
    (send _br :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; geometry_msgs/Twist _twist
       (send _twist :serialize s)
     ;; stoch3_msgs/LegState _fl
       (send _fl :serialize s)
     ;; stoch3_msgs/LegState _fr
       (send _fr :serialize s)
     ;; stoch3_msgs/LegState _bl
       (send _bl :serialize s)
     ;; stoch3_msgs/LegState _br
       (send _br :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; geometry_msgs/Twist _twist
     (send _twist :deserialize buf ptr-) (incf ptr- (send _twist :serialization-length))
   ;; stoch3_msgs/LegState _fl
     (send _fl :deserialize buf ptr-) (incf ptr- (send _fl :serialization-length))
   ;; stoch3_msgs/LegState _fr
     (send _fr :deserialize buf ptr-) (incf ptr- (send _fr :serialization-length))
   ;; stoch3_msgs/LegState _bl
     (send _bl :deserialize buf ptr-) (incf ptr- (send _bl :serialization-length))
   ;; stoch3_msgs/LegState _br
     (send _br :deserialize buf ptr-) (incf ptr- (send _br :serialization-length))
   ;;
   self)
  )

(setf (get stoch3_msgs::QuadrupedRobotState :md5sum-) "c0b1dcdf6adea2ab6ab0d9d2dab7e7de")
(setf (get stoch3_msgs::QuadrupedRobotState :datatype-) "stoch3_msgs/QuadrupedRobotState")
(setf (get stoch3_msgs::QuadrupedRobotState :definition-)
      "Header header
geometry_msgs/Pose pose
geometry_msgs/Twist twist
stoch3_msgs/LegState fl
stoch3_msgs/LegState fr
stoch3_msgs/LegState bl
stoch3_msgs/LegState br

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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

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
================================================================================
MSG: stoch3_msgs/LegState
string name
geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 force
float64 support_probability        # [0, 1] , probability that the leg is a support leg.
                                   # Limit the value to range [0, 1] if it is outside the range.

")



(provide :stoch3_msgs/QuadrupedRobotState "c0b1dcdf6adea2ab6ab0d9d2dab7e7de")


