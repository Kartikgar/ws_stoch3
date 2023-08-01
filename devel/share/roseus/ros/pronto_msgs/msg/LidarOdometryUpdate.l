;; Auto-generated. Do not edit!


(when (boundp 'pronto_msgs::LidarOdometryUpdate)
  (if (not (find-package "PRONTO_MSGS"))
    (make-package "PRONTO_MSGS"))
  (shadow 'LidarOdometryUpdate (find-package "PRONTO_MSGS")))
(unless (find-package "PRONTO_MSGS::LIDARODOMETRYUPDATE")
  (make-package "PRONTO_MSGS::LIDARODOMETRYUPDATE"))

(in-package "ROS")
;;//! \htmlinclude LidarOdometryUpdate.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pronto_msgs::LidarOdometryUpdate
  :super ros::object
  :slots (_header _curr_timestamp _prev_timestamp _relative_transform _covariance ))

(defmethod pronto_msgs::LidarOdometryUpdate
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:curr_timestamp __curr_timestamp) (instance ros::time :init))
    ((:prev_timestamp __prev_timestamp) (instance ros::time :init))
    ((:relative_transform __relative_transform) (instance geometry_msgs::Transform :init))
    ((:covariance __covariance) (make-array 36 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _curr_timestamp __curr_timestamp)
   (setq _prev_timestamp __prev_timestamp)
   (setq _relative_transform __relative_transform)
   (setq _covariance __covariance)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:curr_timestamp
   (&optional __curr_timestamp)
   (if __curr_timestamp (setq _curr_timestamp __curr_timestamp)) _curr_timestamp)
  (:prev_timestamp
   (&optional __prev_timestamp)
   (if __prev_timestamp (setq _prev_timestamp __prev_timestamp)) _prev_timestamp)
  (:relative_transform
   (&rest __relative_transform)
   (if (keywordp (car __relative_transform))
       (send* _relative_transform __relative_transform)
     (progn
       (if __relative_transform (setq _relative_transform (car __relative_transform)))
       _relative_transform)))
  (:covariance
   (&optional __covariance)
   (if __covariance (setq _covariance __covariance)) _covariance)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; time _curr_timestamp
    8
    ;; time _prev_timestamp
    8
    ;; geometry_msgs/Transform _relative_transform
    (send _relative_transform :serialization-length)
    ;; float64[36] _covariance
    (* 8    36)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; time _curr_timestamp
       (write-long (send _curr_timestamp :sec) s) (write-long (send _curr_timestamp :nsec) s)
     ;; time _prev_timestamp
       (write-long (send _prev_timestamp :sec) s) (write-long (send _prev_timestamp :nsec) s)
     ;; geometry_msgs/Transform _relative_transform
       (send _relative_transform :serialize s)
     ;; float64[36] _covariance
     (dotimes (i 36)
       (sys::poke (elt _covariance i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; time _curr_timestamp
     (send _curr_timestamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _curr_timestamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; time _prev_timestamp
     (send _prev_timestamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _prev_timestamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Transform _relative_transform
     (send _relative_transform :deserialize buf ptr-) (incf ptr- (send _relative_transform :serialization-length))
   ;; float64[36] _covariance
   (dotimes (i (length _covariance))
     (setf (elt _covariance i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get pronto_msgs::LidarOdometryUpdate :md5sum-) "f571239a848b48578d24ae00902925c8")
(setf (get pronto_msgs::LidarOdometryUpdate :datatype-) "pronto_msgs/LidarOdometryUpdate")
(setf (get pronto_msgs::LidarOdometryUpdate :definition-)
      "Header header
time curr_timestamp
time prev_timestamp
geometry_msgs/Transform relative_transform
float64[36] covariance


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
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

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
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :pronto_msgs/LidarOdometryUpdate "f571239a848b48578d24ae00902925c8")


